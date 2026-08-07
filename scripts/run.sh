#!/usr/bin/env bash
#
# run.sh — 启动 LVI-SAM-ROS2-Enhanced（LIS + VIS）
#
# 用法：
#   bash scripts/run.sh
#   bash scripts/run.sh robot_description_file:=/path/to/robot.urdf.xacro use_sim_time:=true
#   bash scripts/run.sh lidar_params_file:=/abs/path/params_gazebo_localization.yaml
#
# 说明：激光驱动(livox_ros_driver2)需另行启动，本脚本只拉起 lvi_sam 的 5 个节点。
#
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  echo "[ERR ] 未找到 ROS 2 ($ROS_DISTRO)，请先安装。" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [ ! -f "$WS_ROOT/install/setup.bash" ]; then
  echo "[ERR ] 未找到 install/setup.bash，请先运行 bash scripts/build.sh" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash"

echo "[INFO] 启动 lvi_sam run.launch.py $*"
exec ros2 launch lvi_sam run.launch.py "$@"
