#!/usr/bin/env bash
#
# build.sh — 编译 LVI-SAM-ROS2-Enhanced 工作区
#
#   - 确保 livox_ros_driver2 子模块已初始化
#   - colcon build --symlink-install --packages-up-to lvi_sam
#     （会自动先编 livox_ros_driver2，再编 lvi_sam：LIS + VIS）
#
# 用法： bash scripts/build.sh                         # 全量（LIS + VIS）
#       bash scripts/build.sh --clean                 # 先清理再编
#       bash scripts/build.sh --lidar-only --clean    # 只编 LIS，首轮上车推荐
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
c_warn(){ echo -e "\033[1;33m[WARN]\033[0m  $*"; }

if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  echo "[ERR ] 未找到 ROS 2 ($ROS_DISTRO)，请先安装。" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "/opt/ros/$ROS_DISTRO/setup.bash"

cd "$WS_ROOT"

CLEAN_BUILD=0
BUILD_VISUAL=ON
for arg in "$@"; do
  case "$arg" in
    --clean) CLEAN_BUILD=1 ;;
    --lidar-only) BUILD_VISUAL=OFF ;;
    -h|--help)
      echo "用法: bash scripts/build.sh [--clean] [--lidar-only]"
      exit 0
      ;;
    *)
      echo "[ERR ] 未知参数: $arg" >&2
      exit 2
      ;;
  esac
done

# Ensure the driver dependency is available.  A robot workspace may already
# provide a tested livox_ros_driver2 in a sourced underlay; in that case do not
# force a second source copy into this repository.
if [ ! -f src/livox_ros_driver2/package.xml ]; then
  if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
    _livox_prefix="$(ros2 pkg prefix livox_ros_driver2)"
    c_ok "使用已安装的 livox_ros_driver2: $_livox_prefix"
    c_warn "本仓库的 Livox 子模块未初始化；当前构建将复用已 source 的驱动版本。"
  else
    c_info "初始化子模块 livox_ros_driver2 ..."
    git submodule update --init --recursive
  fi
fi

if [ "$CLEAN_BUILD" = "1" ]; then
  c_info "清理 build/install/log ..."
  rm -rf build install log
fi

# ---------- Orin(AGX Orin/Jetson) 专属：OpenCV 冲突处理 ----------
# JetPack 系统自带 CUDA 版 OpenCV（4.8/4.10），ros-humble 还会装 4.5.4，二者并存时
# cv_bridge 与本项目节点可能因链接不同版本在运行期 ABI 崩溃。
# 策略（默认 KEEP_SYSTEM=0，优先 ROS/cv_bridge 使用的发行版 OpenCV）：
#   - KEEP_SYSTEM=0 时由项目 CMake 自动选择 /usr/lib/<multiarch> 的 ROS 兼容版本；
#   - KEEP_SYSTEM=1 仅用于已针对 /usr/local OpenCV 重编 cv_bridge 的高级环境。
if [ "$IS_ORIN" -eq 1 ]; then
  KEEP_SYSTEM="${KEEP_SYSTEM:-0}"
  if [ -n "${OPENCV_DIR:-}" ] && [ -z "${OpenCV_DIR:-}" ]; then
    c_warn "OPENCV_DIR 已弃用，自动转换为标准变量 OpenCV_DIR。"
    OpenCV_DIR="$OPENCV_DIR"
  fi
  if [ "$KEEP_SYSTEM" = "1" ] && [ -z "${OpenCV_DIR:-}" ]; then
    _pkg_cv="$(pkg-config --variable=libdir opencv4 2>/dev/null || true)"
    _auto_cv=""
    if [ -n "$_pkg_cv" ] && [ -f "$_pkg_cv/cmake/opencv4/OpenCVConfig.cmake" ]; then
      _auto_cv="$_pkg_cv/cmake/opencv4/OpenCVConfig.cmake"
    else
      _auto_cv="$(find /usr/local /opt /usr -name OpenCVConfig.cmake -path '*opencv4*' -print -quit 2>/dev/null || true)"
    fi
    if [ -n "$_auto_cv" ]; then
      OpenCV_DIR="$(dirname "$_auto_cv")"
      c_info "AGX Orin：使用 OpenCV_DIR=$OpenCV_DIR"
    else
      c_warn "AGX Orin：未自动找到 OpenCVConfig.cmake；若出现 ABI 问题，请手动设置 OpenCV_DIR。"
    fi
  else
    c_info "AGX Orin：KEEP_SYSTEM=$KEEP_SYSTEM，OpenCV_DIR=${OpenCV_DIR:-（未设，使用 CMake 默认查找）}"
  fi
fi

# ---------- ccache（加速重复编译，尤其 Orin） ----------
if command -v ccache >/dev/null 2>&1; then
  c_info "已启用 ccache（编译器启动器）。"
fi

# ---------- Orin 并行度上限（避免 OOM / 系统卡死） ----------
COLCON_JOBS="$(nproc)"
if [ "$IS_ORIN" -eq 1 ]; then
  COLCON_JOBS=4
  c_info "AGX Orin：colcon 并行编译限制为 $COLCON_JOBS 个包（默认全核易 OOM / 系统无响应）。"
fi

c_info "colcon build（packages-up-to lvi_sam）..."
CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release "-DBUILD_VISUAL=$BUILD_VISUAL")
if command -v ccache >/dev/null 2>&1; then
  CMAKE_ARGS+=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
fi
if [ -n "${OpenCV_DIR:-}" ]; then
  CMAKE_ARGS+=("-DOpenCV_DIR=$OpenCV_DIR")
fi
if [ "$IS_ORIN" -eq 1 ] && [ "${KEEP_SYSTEM:-0}" = "1" ]; then
  CMAKE_ARGS+=(-DLVI_SAM_PREFER_ROS_OPENCV=OFF)
fi
colcon build --symlink-install --packages-up-to lvi_sam \
  --parallel-workers "$COLCON_JOBS" \
  --cmake-args "${CMAKE_ARGS[@]}"

c_ok "编译完成。source install/setup.bash 后即可运行：bash scripts/run.sh"
