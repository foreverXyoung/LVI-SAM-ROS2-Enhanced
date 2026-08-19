#!/usr/bin/env bash
#
# build.sh — 编译 LVI-SAM-ROS2-Enhanced 工作区
#
#   - 仓库位于 <workspace>/src 时自动使用上层 colcon 工作区
#   - 优先复用已安装的 livox_ros_driver2；缺失时才构建子模块
#
# 用法： bash scripts/build.sh                         # 全量（LIS + VIS）
#       bash scripts/build.sh --clean                 # 先清理再编
#       bash scripts/build.sh --lidar-only --clean    # 只编 LIS，首轮上车推荐
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="${LVI_SAM_WORKSPACE_ROOT:-$REPO_ROOT}"
if [ -z "${LVI_SAM_WORKSPACE_ROOT:-}" ] && \
   [ "$(basename "$(dirname "$REPO_ROOT")")" = "src" ]; then
  WS_ROOT="$(cd "$REPO_ROOT/../.." && pwd)"
fi
TARGET_ROS_DISTRO="${ROS_DISTRO:-humble}"

# 平台检测
ARCH="$(uname -m)"
IS_ORIN=0
if [ "$ARCH" = "aarch64" ]; then IS_ORIN=1; fi

c_info(){ echo -e "\033[1;34m[INFO]\033[0m  $*"; }
c_ok(){   echo -e "\033[1;32m[ OK ]\033[0m  $*"; }
c_warn(){ echo -e "\033[1;33m[WARN]\033[0m  $*"; }

if [ ! -f "/opt/ros/$TARGET_ROS_DISTRO/setup.bash" ]; then
  echo "[ERR ] 未找到 ROS 2 ($TARGET_ROS_DISTRO)，请先安装。" >&2
  exit 1
fi
if ! command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  # ROS 的 setup.bash 可能引用未设置的变量；在 set -u 下 source 前临时关闭 nounset。
  set +u
  source "/opt/ros/$TARGET_ROS_DISTRO/setup.bash"
  set -u
else
  c_info "保留当前已 source 的 ROS/工作区环境，以便复用平台驱动。"
fi
if [ "$WS_ROOT" != "$REPO_ROOT" ] && \
   [ -f "$WS_ROOT/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  set +u
  source "$WS_ROOT/install/setup.bash"
  set -u
  c_info "已自动 source 机器人工作区: $WS_ROOT/install/setup.bash"
fi

cd "$REPO_ROOT"

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

c_info "核验 LIS/VIS 配置与接口契约 ..."
VALIDATOR_ARGS=(--config-dir "$REPO_ROOT/src/lvi_sam/config")
if [ "$BUILD_VISUAL" = "OFF" ]; then
  VALIDATOR_ARGS+=(--lidar-only)
fi
python3 "$REPO_ROOT/src/lvi_sam/scripts/validate_config.py" "${VALIDATOR_ARGS[@]}"
c_ok "配置与接口契约核验通过。"

# Ensure the driver dependency is available.  A robot workspace may already
# provide a tested livox_ros_driver2 in a sourced underlay; in that case do not
# force a second source copy into this repository.
COLCON_PACKAGE_ARGS=(--packages-up-to lvi_sam)
COLCON_BASE_PATH_ARGS=(
  --base-paths "$REPO_ROOT/src/lvi_sam" "$REPO_ROOT/src/livox_ros_driver2"
)
if command -v ros2 >/dev/null 2>&1 && \
   ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
  _livox_prefix="$(ros2 pkg prefix livox_ros_driver2)"
  c_ok "使用已安装的 livox_ros_driver2: $_livox_prefix"
  c_warn "仅构建 lvi_sam，避免重复编译机器人工作区中已验证的驱动。"
  COLCON_PACKAGE_ARGS=(--packages-select lvi_sam)
  COLCON_BASE_PATH_ARGS=(--base-paths "$REPO_ROOT/src/lvi_sam")
elif [ -f "$REPO_ROOT/src/livox_ros_driver2/package.xml" ]; then
  c_info "未发现已安装驱动；将同时构建仓库内 livox_ros_driver2 子模块。"
else
  c_info "初始化子模块 livox_ros_driver2 ..."
  git -C "$REPO_ROOT" submodule update --init --recursive
fi

if [ "$CLEAN_BUILD" = "1" ]; then
  c_info "仅清理 lvi_sam 的构建与安装产物 ..."
  for _build_output in "$WS_ROOT/build/lvi_sam" "$WS_ROOT/install/lvi_sam"; do
    case "$_build_output" in
      "$WS_ROOT"/build/lvi_sam|"$WS_ROOT"/install/lvi_sam)
        rm -rf -- "$_build_output"
        ;;
      *)
        echo "[ERR ] 拒绝清理非 lvi_sam 产物路径: $_build_output" >&2
        exit 1
        ;;
    esac
  done
fi

# ---------- OpenCV 选择 ----------
# VIS 使用包内图像适配层，不再链接 cv_bridge。CMake 只选择一套 OpenCV；如机器
# 上存在多个 OpenCVConfig.cmake，可用标准变量 OpenCV_DIR 显式指定其中一个。
if [ -n "${OPENCV_DIR:-}" ] && [ -z "${OpenCV_DIR:-}" ]; then
  c_warn "OPENCV_DIR 已弃用，自动转换为标准变量 OpenCV_DIR。"
  OpenCV_DIR="$OPENCV_DIR"
fi
if [ -n "${KEEP_SYSTEM:-}" ]; then
  c_warn "KEEP_SYSTEM 已不再需要：LVI-SAM VIS 已移除 cv_bridge 二进制依赖。"
fi
if [ "$IS_ORIN" -eq 1 ]; then
  c_info "AGX Orin：VIS 使用内部图像适配层；OpenCV_DIR=${OpenCV_DIR:-（未设，使用 CMake 默认查找）}"
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

c_info "colcon 工作区: $WS_ROOT"
c_info "colcon build (${COLCON_PACKAGE_ARGS[*]}) ..."
CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release "-DBUILD_VISUAL=$BUILD_VISUAL")
if command -v ccache >/dev/null 2>&1; then
  CMAKE_ARGS+=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
fi
if [ -n "${OpenCV_DIR:-}" ]; then
  CMAKE_ARGS+=("-DOpenCV_DIR=$OpenCV_DIR")
fi
cd "$WS_ROOT"
colcon build "${COLCON_BASE_PATH_ARGS[@]}" --symlink-install \
  "${COLCON_PACKAGE_ARGS[@]}" \
  --parallel-workers "$COLCON_JOBS" \
  --cmake-args "${CMAKE_ARGS[@]}"

c_ok "编译完成。环境入口：$WS_ROOT/install/setup.bash；可直接运行 bash scripts/run.sh"
