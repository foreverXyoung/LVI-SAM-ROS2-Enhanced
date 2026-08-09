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

# 平台检测
ARCH="$(uname -m)"
IS_ORIN=0
if [ "$ARCH" = "aarch64" ]; then IS_ORIN=1; fi

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

# ---------- Orin(AGX Orin/Jetson) 专属：OpenCV 冲突处理 ----------
# JetPack 系统自带 CUDA 版 OpenCV（4.8/4.10），ros-humble 还会装 4.5.4，二者并存时
# cv_bridge 与本项目节点可能因链接不同版本在运行期 ABI 崩溃。
# 策略（默认 KEEP_SYSTEM=1，沿用系统 CUDA 版 OpenCV）：
#   - 自动查找系统 opencv4 的 OpenCVConfig.cmake，导出 OPENCV_DIR 让 cv_bridge 与本项目一致；
#   - 若自动查找失败或想改用 4.5.4，可设 OPENCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4（x86）
#     或 apt 版路径，或设 KEEP_SYSTEM=0 交给 apt 的 4.5.4。
if [ "$IS_ORIN" -eq 1 ]; then
  KEEP_SYSTEM="${KEEP_SYSTEM:-1}"
  if [ "$KEEP_SYSTEM" = "1" ] && [ -z "${OPENCV_DIR:-}" ]; then
    _auto_cv="$(find /usr -name OpenCVConfig.cmake -path '*opencv4*' 2>/dev/null | head -n1)"
    if [ -n "$_auto_cv" ]; then
      export OPENCV_DIR="$(dirname "$_auto_cv")"
      c_info "AGX Orin：使用系统 OpenCV（CUDA 版），OPENCV_DIR=$OPENCV_DIR"
    else
      c_warn "AGX Orin：未自动找到系统 OpenCVConfig.cmake，保持默认查找；若运行期报 cv:: 符号错误，请手动设 OPENCV_DIR。"
    fi
  else
    c_info "AGX Orin：KEEP_SYSTEM=$KEEP_SYSTEM，OPENCV_DIR=${OPENCV_DIR:-（未设，使用 CMake 默认查找）}"
  fi
fi

# ---------- ccache（加速重复编译，尤其 Orin） ----------
if command -v ccache >/dev/null 2>&1; then
  export CMAKE_C_COMPILER_LAUNCHER=ccache
  export CMAKE_CXX_COMPILER_LAUNCHER=ccache
  c_info "已启用 ccache（编译器启动器）。"
fi

# ---------- Orin 并行度上限（避免 OOM / 系统卡死） ----------
COLCON_JOBS="$(nproc)"
if [ "$IS_ORIN" -eq 1 ]; then
  COLCON_JOBS=4
  c_info "AGX Orin：colcon 并行编译限制为 $COLCON_JOBS 个包（默认全核易 OOM / 系统无响应）。"
fi

c_info "colcon build（packages-up-to lvi_sam）..."
colcon build --symlink-install --packages-up-to lvi_sam \
  --parallel-workers "$COLCON_JOBS" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

c_ok "编译完成。source install/setup.bash 后即可运行：bash scripts/run.sh"
