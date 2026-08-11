#!/usr/bin/env python3
import argparse
import math
import os
import shutil
import statistics
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from pyproj import Transformer
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GpsSampleReader(Node):
    def __init__(self, topic: str, sample_count: int):
        super().__init__("gps_to_init_guess_once")
        self.sample_count = sample_count
        self.messages = []
        self.sub = self.create_subscription(
            NavSatFix,
            topic,
            self.callback,
            10
        )

    def callback(self, msg: NavSatFix):
        if len(self.messages) < self.sample_count:
            self.messages.append(msg)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def gps_to_map_xy(lat: float, lon: float, geo: dict):
    """
    GPS 经纬度 -> UTM -> LIO-SAM 地图坐标 x/y

    map_geo.yaml 里定义的是：

        UTM = R(theta) * map + t

    所以反算：

        map = R(-theta) * (UTM - t)
    """
    epsg = int(geo["utm_epsg"])
    theta = float(geo["theta_rad"])
    t_easting = float(geo["t_easting"])
    t_northing = float(geo["t_northing"])

    transformer = Transformer.from_crs(
        "EPSG:4326",
        f"EPSG:{epsg}",
        always_xy=True
    )

    easting, northing = transformer.transform(lon, lat)

    d_e = easting - t_easting
    d_n = northing - t_northing

    c = math.cos(theta)
    s = math.sin(theta)

    # R(-theta) * [dE, dN]
    x_map = c * d_e + s * d_n
    y_map = -s * d_e + c * d_n

    return x_map, y_map, easting, northing


def make_init_guess_line(x: float, y: float, z: float, yaw: float, order: str, comment: str) -> str:
    yaw = normalize_angle(yaw)

    if order == "rpyxyz":
        # 你的 LIO-SAM_SC_LOC_MID360_ROS2 当前格式：
        # init_guess: [roll, pitch, yaw, x, y, z]
        values = f"[0.0, 0.0, {yaw:.6f}, {x:.6f}, {y:.6f}, {z:.6f}]"
    elif order == "xyzrpy":
        # 备用格式：
        # init_guess: [x, y, z, roll, pitch, yaw]
        values = f"[{x:.6f}, {y:.6f}, {z:.6f}, 0.0, 0.0, {yaw:.6f}]"
    else:
        raise ValueError(f"Unknown init_guess order: {order}")

    if comment:
        return f"init_guess: {values} {comment}"
    else:
        return f"init_guess: {values}"


def update_init_guess(params_path: str, x: float, y: float, z: float, yaw: float, order: str):
    path = Path(params_path)

    if not path.exists():
        raise FileNotFoundError(f"Params file not found: {path}")

    text = path.read_text(encoding="utf-8")

    yaw = normalize_angle(yaw)

    if order == "rpyxyz":
        values = f"[0.0, 0.0, {yaw:.6f}, {x:.6f}, {y:.6f}, {z:.6f}]"
    elif order == "xyzrpy":
        values = f"[{x:.6f}, {y:.6f}, {z:.6f}, 0.0, 0.0, {yaw:.6f}]"
    else:
        raise ValueError(f"Unknown init_guess order: {order}")

    lines = text.splitlines()
    changed = False
    new_line_text = ""

    for i, line in enumerate(lines):
        stripped = line.lstrip()

        # 跳过注释行
        if stripped.startswith("#"):
            continue

        # 只修改真正生效的 init_guess 行
        if stripped.startswith("init_guess:"):
            indent = line[:len(line) - len(stripped)]

            if "#" in line:
                comment = line[line.index("#"):]
            else:
                comment = "# [roll,pitch,yaw,x,y,z], GPS init_guess"

            new_line_text = f"{indent}init_guess: {values}   {comment}"
            lines[i] = new_line_text
            changed = True
            break

    if not changed:
        raise RuntimeError(
            f"Active init_guess line not found in {path}. "
            f"Make sure there is an uncommented line like: init_guess: [...]"
        )

    updated_text = "\n".join(lines) + "\n"
    try:
        updated_document = yaml.safe_load(updated_text)
        updated_document["/**"]["ros__parameters"]["Loc"]["init_guess"]
    except (yaml.YAMLError, KeyError, TypeError) as exc:
        raise RuntimeError(
            f"Refusing to write invalid ROS parameter YAML: {exc}"
        ) from exc

    backup_path = Path(
        str(path) +
        f".bak_gpsinit_{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns()}"
    )
    shutil.copy2(path, backup_path)

    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            prefix=f".{path.name}.",
            suffix=".tmp",
            dir=path.parent,
            delete=False,
        ) as temporary_file:
            temporary_file.write(updated_text)
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
            temporary_path = Path(temporary_file.name)
        os.replace(temporary_path, path)
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)

    return new_line_text.strip(), str(backup_path)


def read_gps_samples(gps_topic: str, timeout: float, sample_count: int):
    rclpy.init()
    node = GpsSampleReader(gps_topic, sample_count)

    start_time = time.time()

    while time.time() - start_time < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(node.messages) >= sample_count:
            break

    messages = list(node.messages)
    node.destroy_node()
    rclpy.shutdown()

    if len(messages) < sample_count:
        raise RuntimeError(
            f"Only {len(messages)}/{sample_count} GPS samples received from "
            f"{gps_topic} within {timeout} seconds"
        )

    return messages


def validate_gps_sample(msg: NavSatFix, min_status: int,
                        max_horizontal_std: float,
                        allow_unknown_covariance: bool):
    if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
        raise RuntimeError("GPS contains a non-finite latitude/longitude")
    if msg.latitude == 0.0 and msg.longitude == 0.0:
        raise RuntimeError("Invalid GPS: latitude and longitude are both 0")
    if msg.status.status < min_status:
        raise RuntimeError(
            f"GPS status too low: {msg.status.status}, required >= {min_status}"
        )

    if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        if not allow_unknown_covariance:
            raise RuntimeError(
                "GPS covariance is unknown; use --allow-unknown-covariance "
                "only after checking the receiver contract"
            )
        return

    variance_e = float(msg.position_covariance[0])
    variance_n = float(msg.position_covariance[4])
    if (not math.isfinite(variance_e) or not math.isfinite(variance_n)
            or variance_e <= 0.0 or variance_n <= 0.0):
        raise RuntimeError("GPS horizontal covariance is invalid")
    horizontal_std = math.sqrt(max(variance_e, variance_n))
    if horizontal_std > max_horizontal_std:
        raise RuntimeError(
            f"GPS horizontal std {horizontal_std:.3f} m exceeds "
            f"{max_horizontal_std:.3f} m"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Use current /gps/fix to update LIO-SAM localization init_guess."
    )

    parser.add_argument(
        "--geo",
        default="/tmp/lvi_sam_maps/map_geo.yaml",
        help="Path to map_geo.yaml (override at runtime; machine-agnostic default)"
    )

    parser.add_argument(
        "--params",
        default="/tmp/lvi_sam_maps/params_localization.yaml",
        help="Path to params_localization.yaml (override at runtime)"
    )

    parser.add_argument(
        "--params-active",
        default="/tmp/lvi_sam_maps/params.yaml",
        help="Path to active params.yaml (override at runtime)"
    )

    parser.add_argument(
        "--update-active",
        action="store_true",
        help="Also update active params.yaml"
    )

    parser.add_argument(
        "--gps-topic",
        default="/gps/fix",
        help="GPS NavSatFix topic"
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Timeout seconds for waiting GPS message"
    )

    parser.add_argument(
        "--z",
        type=float,
        default=0.0,
        help="Initial z in LIO-SAM map frame"
    )

    parser.add_argument(
        "--yaw",
        type=float,
        default=0.0,
        help="Initial yaw in LIO-SAM map frame, radian"
    )

    parser.add_argument(
        "--order",
        choices=["rpyxyz", "xyzrpy"],
        default="rpyxyz",
        help="init_guess order. rpyxyz means [roll,pitch,yaw,x,y,z]"
    )

    parser.add_argument(
        "--min-status",
        type=int,
        default=0,
        help="Minimum acceptable NavSatFix status; RTK fix values are driver-specific."
    )

    parser.add_argument(
        "--samples",
        type=int,
        default=5,
        help="Number of consecutive GPS samples used for the robust median"
    )

    parser.add_argument(
        "--max-horizontal-std",
        type=float,
        default=0.5,
        help="Maximum accepted horizontal standard deviation in metres"
    )

    parser.add_argument(
        "--max-sample-spread",
        type=float,
        default=0.75,
        help="Maximum XY spread among accepted initialization samples in metres"
    )

    parser.add_argument(
        "--allow-unknown-covariance",
        action="store_true",
        help="Allow NavSatFix with unknown covariance (not recommended)"
    )

    args = parser.parse_args()

    geo_path = Path(args.geo)

    if not geo_path.exists():
        raise FileNotFoundError(f"map_geo.yaml not found: {geo_path}")

    with geo_path.open("r") as f:
        geo = yaml.safe_load(f)

    if args.samples < 1:
        raise ValueError("--samples must be at least 1")
    if args.max_horizontal_std <= 0.0 or args.max_sample_spread < 0.0:
        raise ValueError("GPS quality thresholds must be positive")

    messages = read_gps_samples(args.gps_topic, args.timeout, args.samples)
    for msg in messages:
        validate_gps_sample(
            msg, args.min_status, args.max_horizontal_std,
            args.allow_unknown_covariance
        )

    projected = [gps_to_map_xy(float(msg.latitude), float(msg.longitude), geo)
                 for msg in messages]
    x_values = [item[0] for item in projected]
    y_values = [item[1] for item in projected]
    x_map = statistics.median(x_values)
    y_map = statistics.median(y_values)
    max_spread = max(math.hypot(x - x_map, y - y_map)
                     for x, y in zip(x_values, y_values))
    if max_spread > args.max_sample_spread:
        raise RuntimeError(
            f"GPS sample spread {max_spread:.3f} m exceeds "
            f"{args.max_sample_spread:.3f} m"
        )

    lat = statistics.median(float(msg.latitude) for msg in messages)
    lon = statistics.median(float(msg.longitude) for msg in messages)
    _, _, easting, northing = gps_to_map_xy(lat, lon, geo)

    new_line, backup_path = update_init_guess(
        args.params,
        x_map,
        y_map,
        args.z,
        args.yaw,
        args.order
    )

    print("========== GPS to LIO-SAM init_guess ==========")
    print(f"GPS topic      : {args.gps_topic}")
    print(f"GPS samples    : {len(messages)}")
    print(f"GPS status min : {min(msg.status.status for msg in messages)}")
    print(f"Sample spread  : {max_spread:.3f} m")
    print(f"GPS lat/lon    : {lat:.12f}, {lon:.12f}")
    print(f"UTM E/N        : {easting:.3f}, {northing:.3f}")
    print(f"Map x/y        : {x_map:.3f}, {y_map:.3f}")
    print(f"Yaw            : {normalize_angle(args.yaw):.6f} rad")
    print(f"Params file    : {args.params}")
    print(f"Backup file    : {backup_path}")
    print(f"New init_guess : {new_line}")

    if args.update_active:
        new_line_active, backup_active = update_init_guess(
            args.params_active,
            x_map,
            y_map,
            args.z,
            args.yaw,
            args.order
        )

        print("")
        print("========== Active params.yaml also updated ==========")
        print(f"Active params  : {args.params_active}")
        print(f"Backup file    : {backup_active}")
        print(f"New init_guess : {new_line_active}")


if __name__ == "__main__":
    main()
