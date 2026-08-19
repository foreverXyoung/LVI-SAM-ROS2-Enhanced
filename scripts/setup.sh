#!/usr/bin/env bash
#
# setup.sh — 一键部署 LVI-SAM-ROS2-Enhanced
#
#   1) 复用已 source 的 livox_ros_driver2，缺失时才初始化子模块
#   2) 安装全部依赖（apt + GTSAM 源码 + Livox-SDK2 源码 + rosdep）
#   3) 编译工作区
#
# 用法：  bash scripts/setup.sh
#
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT"

echo "============================================================"
echo " LVI-SAM-ROS2-Enhanced 一键部署"
echo " 工作区: $WS_ROOT"
echo "============================================================"

# 1) 驱动依赖。机器人总工作区已提供并 source 驱动时，不再拉取第二份源码。
echo "[1/3] 检查 livox_ros_driver2 ..."
ROS_DISTRO="${ROS_DISTRO:-humble}"
if ! command -v ros2 >/dev/null 2>&1 && \
   [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/$ROS_DISTRO/setup.bash"
fi
if command -v ros2 >/dev/null 2>&1 && \
     ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
  echo "[ OK ] 复用已 source 的 livox_ros_driver2: $(ros2 pkg prefix livox_ros_driver2)"
elif [ -f src/livox_ros_driver2/package.xml ]; then
  echo "[ OK ] 使用仓库内 livox_ros_driver2 子模块"
else
  echo "[INFO] 未发现已安装驱动，初始化 livox_ros_driver2 子模块 ..."
  git submodule update --init --recursive
fi

# 2) 依赖
echo "[2/3] 安装依赖（可能需要 sudo，耗时较长）..."
bash "$WS_ROOT/scripts/install_deps.sh"

# 3) 编译
echo "[3/3] 编译工作区 ..."
bash "$WS_ROOT/scripts/build.sh"

echo "============================================================"
echo " 部署完成！"
echo " 运行:  bash scripts/run.sh robot_description_file:=/path/to/robot.urdf.xacro"
echo " 详见:  docs/ENVIRONMENT.md  docs/USAGE.md"
echo "============================================================"
