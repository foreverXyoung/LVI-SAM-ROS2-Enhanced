#!/usr/bin/env bash
#
# build.sh — 编译 LVI-SAM-ROS2-Enhanced 工作区
#
#   - 确保 livox_ros_driver2 子模块已初始化
#   - colcon build --symlink-install --packages-up-to lvi_sam
#     （会自动先编 livox_ros_driver2，再编 lvi_sam：LIS + VIS）
#
# 用法：  bash scripts/build.sh            # 全量
#        bash scripts/build.sh --clean    # 先清理再编
#
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

c_info(){ echo -e "\033[1;34m[INFO]\033[0m  $*"; }
c_ok(){   echo -e "\033[1;32m[ OK ]\033[0m  $*"; }

if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  echo "[ERR ] 未找到 ROS 2 ($ROS_DISTRO)，请先安装。" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "/opt/ros/$ROS_DISTRO/setup.bash"

cd "$WS_ROOT"

# 确保子模块
if [ ! -f src/livox_ros_driver2/package.xml ]; then
  c_info "初始化子模块 livox_ros_driver2 ..."
  git submodule update --init --recursive
fi

if [ "${1:-}" = "--clean" ]; then
  c_info "清理 build/install/log ..."
  rm -rf build install log
fi

c_info "colcon build（packages-up-to lvi_sam）..."
colcon build --symlink-install --packages-up-to lvi_sam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

c_ok "编译完成。source install/setup.bash 后即可运行：bash scripts/run.sh"
