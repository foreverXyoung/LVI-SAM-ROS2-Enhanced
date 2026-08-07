#!/usr/bin/env python3
import argparse
import math
import re
import time
from pathlib import Path

import rclpy
import yaml
from pyproj import Transformer
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GpsOnceReader(Node):
    def __init__(self, topic: str):
        super().__init__("gps_to_init_guess_once")
        self.msg = None
        self.sub = self.create_subscription(
            NavSatFix,
            topic,
            self.callback,
            10
        )

    def callback(self, msg: NavSatFix):
        self.msg = msg


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
    from pathlib import Path
    import time

    path = Path(params_path)

    if not path.exists():
        raise FileNotFoundError(f"Params file not found: {path}")

    text = path.read_text()
    backup_path = str(path) + f".bak_gpsinit_{time.strftime('%Y%m%d_%H%M%S')}"
    Path(backup_path).write_text(text)

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

    path.write_text("\n".join(lines) + "\n")

    return new_line_text.strip(), backup_path


def read_one_gps(gps_topic: str, timeout: float):
    rclpy.init()
    node = GpsOnceReader(gps_topic)

    msg = None
    start_time = time.time()

    while time.time() - start_time < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.msg is not None:
            msg = node.msg
            break

    node.destroy_node()
    rclpy.shutdown()

    if msg is None:
        raise RuntimeError(f"No GPS message received from {gps_topic} within {timeout} seconds")

    return msg


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
        help="Minimum acceptable NavSatFix status. Use 2 if you only want RTK fixed-like status."
    )

    args = parser.parse_args()

    geo_path = Path(args.geo)

    if not geo_path.exists():
        raise FileNotFoundError(f"map_geo.yaml not found: {geo_path}")

    with geo_path.open("r") as f:
        geo = yaml.safe_load(f)

    msg = read_one_gps(args.gps_topic, args.timeout)

    if msg.latitude == 0.0 and msg.longitude == 0.0:
        raise RuntimeError("Invalid GPS: latitude and longitude are both 0")

    if msg.status.status < args.min_status:
        raise RuntimeError(
            f"GPS status too low: {msg.status.status}, required >= {args.min_status}"
        )

    lat = float(msg.latitude)
    lon = float(msg.longitude)

    x_map, y_map, easting, northing = gps_to_map_xy(lat, lon, geo)

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
    print(f"GPS status     : {msg.status.status}")
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