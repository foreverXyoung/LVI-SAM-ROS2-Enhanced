#!/usr/bin/env python3
"""Verify the phase-1 LVI-SAM localization status contract at runtime.

This is an integration check, not a state-machine controller. It observes the
structured status topic and (for the force case) calls the existing service;
it never changes matcher, TF, odometry, or relocalization parameters.
"""

import argparse
import sys
import time

import rclpy
from lvi_sam_msgs.msg import LocalizationStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


STATUS_TOPIC = "/lio_sam/localization/status"
LEGACY_TOPIC = "/lio_sam/localization/state"
FORCE_SERVICE = "/lio_sam/localization/force_relocalize"


class StatusVerifier(Node):
    def __init__(self) -> None:
        super().__init__("lvi_sam_localization_status_verifier")
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status = None
        self.legacy_state = None
        self.status_history = []
        self.create_subscription(
            LocalizationStatus, STATUS_TOPIC, self._status_callback, status_qos
        )
        self.create_subscription(
            String, LEGACY_TOPIC, self._legacy_callback, 10
        )
        self.force_client = self.create_client(Trigger, FORCE_SERVICE)

    def _status_callback(self, message: LocalizationStatus) -> None:
        self.status = message
        self.status_history.append(message)
        self.status_history = self.status_history[-100:]

    def _legacy_callback(self, message: String) -> None:
        self.legacy_state = message.data

    def wait_for(self, predicate, timeout: float, description: str) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.status is not None and predicate(self.status):
                return True
        self.get_logger().error(
            "Timed out waiting for %s (last=%s legacy=%s)",
            description,
            self._format_status(self.status),
            self.legacy_state,
        )
        return False

    @staticmethod
    def _format_status(status) -> str:
        if status is None:
            return "<none>"
        return (
            f"state={status.state_name}({status.state}) "
            f"mode={status.mode} map_ready={status.map_ready} "
            f"pose_valid={status.pose_valid} seq={status.transition_sequence}"
        )

    def print_status(self) -> None:
        self.get_logger().info(
            "status=%s legacy=%s",
            self._format_status(self.status),
            self.legacy_state,
        )

    def call_force_relocalize(self, timeout: float) -> bool:
        if not self.force_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("Service %s is not available", FORCE_SERVICE)
            return False
        future = self.force_client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                response = future.result()
                if response is None or not response.success:
                    self.get_logger().error(
                        "force_relocalize failed: %s",
                        "<no response>" if response is None else response.message,
                    )
                    return False
                self.get_logger().info("force_relocalize accepted: %s", response.message)
                return True
        self.get_logger().error("Timed out waiting for force_relocalize response")
        return False


def verify_mapping(verifier: StatusVerifier, timeout: float) -> bool:
    return verifier.wait_for(
        lambda status: (
            status.mode == LocalizationStatus.MODE_MAPPING
            and status.state == LocalizationStatus.MAPPING
            and not status.map_ready
        ),
        timeout,
        "mapping status",
    )


def verify_localization(
    verifier: StatusVerifier, timeout: float, require_tracking: bool
) -> bool:
    if not verifier.wait_for(
        lambda status: (
            status.mode == LocalizationStatus.MODE_LOCALIZATION
            and status.map_ready
            and status.state
            in (LocalizationStatus.RELOCALIZING, LocalizationStatus.TRACKING)
        ),
        timeout,
        "localization status",
    ):
        return False
    if not require_tracking:
        return True
    return verifier.wait_for(
        lambda status: (
            status.mode == LocalizationStatus.MODE_LOCALIZATION
            and status.state == LocalizationStatus.TRACKING
            and status.pose_valid
        ),
        timeout,
        "TRACKING status",
    )


def verify_lost(verifier: StatusVerifier, timeout: float) -> bool:
    if not verifier.wait_for(
        lambda status: (
            status.state == LocalizationStatus.LOST
            or any(
                item.state == LocalizationStatus.LOST
                for item in verifier.status_history
            )
        ),
        timeout,
        "latched LOST status",
    ):
        return False
    return verifier.wait_for(
        lambda status: status.state == LocalizationStatus.RELOCALIZING,
        timeout,
        "RELOCALIZING after LOST",
    )


def verify_force(verifier: StatusVerifier, timeout: float) -> bool:
    if not verifier.call_force_relocalize(timeout):
        return False
    return verifier.wait_for(
        lambda status: (
            status.mode == LocalizationStatus.MODE_LOCALIZATION
            and status.state == LocalizationStatus.RELOCALIZING
            and status.relocalization_active
        ),
        timeout,
        "RELOCALIZING after force_relocalize",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--case",
        choices=("mapping", "localization", "lost", "force_relocalize"),
        required=True,
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--allow-uninitialized",
        action="store_true",
        help="For localization, accept RELOCALIZING without waiting for TRACKING.",
    )
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    rclpy.init()
    verifier = StatusVerifier()
    try:
        if args.case == "mapping":
            passed = verify_mapping(verifier, args.timeout)
        elif args.case == "localization":
            passed = verify_localization(
                verifier, args.timeout, not args.allow_uninitialized
            )
        elif args.case == "lost":
            passed = verify_lost(verifier, args.timeout)
        else:
            passed = verify_force(verifier, args.timeout)
        verifier.print_status()
        return 0 if passed else 1
    finally:
        verifier.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
