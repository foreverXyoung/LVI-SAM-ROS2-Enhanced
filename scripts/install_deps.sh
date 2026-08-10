#!/usr/bin/env bash
#
# install_deps.sh — 安装 LVI-SAM-ROS2-Enhanced 的全部依赖
#
#   - apt / ROS 依赖（rosdep）
#   - GTSAM（源码编译，Ubuntu 22.04 无 apt 包）
#   - Livox Lidar SDK（源码编译，预装到 /usr/local）
#
# 设计目标：可重复执行（已安装的步骤自动跳过），适合 CI 与干净部署。
# 用法：  bash scripts/install_deps.sh
#
set -euo pipefail

# ---------- 基础变量 ----------
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SUDO=""
if [ "$(id -u)" -ne 0 ]; then SUDO="sudo"; fi

# 平台检测：aarch64 -> 多为 AGX Orin / Jetson，启用 Orin 专属提示与限制
ARCH="$(uname -m)"
IS_ORIN=0
if [ "$ARCH" = "aarch64" ]; then IS_ORIN=1; fi

# 终端颜色
c_info(){ echo -e "\033[1;34m[INFO]\033[0m  $*"; }
c_ok(){   echo -e "\033[1;32m[ OK ]\033[0m  $*"; }
c_warn(){ echo -e "\033[1;33m[WARN]\033[0m  $*"; }
c_err(){  echo -e "\033[1;31m[ERR ]\033[0m  $*"; }

# ---------- 0) 检测 ROS 2 环境 ----------
c_info "检测 ROS 2 ($ROS_DISTRO) 环境 ..."
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  # shellcheck disable=SC1091
  # ROS 的 setup.bash 会引用未设置的变量（如 AMENT_TRACE_SETUP_FILES），
  # 在 set -u 下会报“未绑定的变量”而中止；临时关闭 nounset 再 source。
  set +u
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  set -u
  c_ok "已 source /opt/ros/$ROS_DISTRO/setup.bash"
else
  c_err "未找到 ROS 2 ($ROS_DISTRO)。请先按 docs/ENVIRONMENT.md 安装 ROS 2 Humble。"
  exit 1
fi

# ---------- 1) apt 基础包 ----------
c_info "安装 apt / 编译基础包 ..."
$SUDO apt-get update -y
$SUDO apt-get install -y --no-install-recommends \
  build-essential cmake git wget curl ca-certificates ccache \
  libpcl-dev libopencv-dev libeigen3-dev libboost-all-dev libceres-dev \
  python3-pip python3-yaml python3-pyproj python3-rosdep python3-colcon-common-extensions \
  ros-"$ROS_DISTRO"-desktop ros-"$ROS_DISTRO"-pcl-ros \
  ros-"$ROS_DISTRO"-pcl-conversions ros-"$ROS_DISTRO"-tf2* \
  ros-"$ROS_DISTRO"-robot-state-publisher ros-"$ROS_DISTRO"-xacro

# Orin 上启用 ccache（加速重复编译），通过 CCACHE_DIR 可持久化
if [ "$IS_ORIN" -eq 1 ]; then
  c_info "检测到 aarch64(AGX Orin/Jetson)：ccache 已安装，建议 export CCACHE_DIR=/mnt/<外存>/ccache 持久化。"
  export CCACHE_DIR="${CCACHE_DIR:-${HOME}/.ccache}"
  export CCACHE_MAXSIZE="${CCACHE_MAXSIZE:-10G}"
  c_info "ccache 配置：CCACHE_DIR=$CCACHE_DIR CCACHE_MAXSIZE=$CCACHE_MAXSIZE（可用 ccache -z 查看命中率）"
fi

# ---------- 2) GTSAM（源码编译，可跳过） ----------
GTSAM_VERSION_FILE="/usr/local/lib/cmake/GTSAM/GTSAMConfigVersion.cmake"
INSTALLED_GTSAM_VERSION=""
if [ -f "$GTSAM_VERSION_FILE" ]; then
  INSTALLED_GTSAM_VERSION="$(grep -m1 -E 'set\(PACKAGE_VERSION "[^"]+"' "$GTSAM_VERSION_FILE" | cut -d'"' -f2 || true)"
fi
if [ "$INSTALLED_GTSAM_VERSION" = "4.0.3" ] || [[ "$INSTALLED_GTSAM_VERSION" == 4.1.* ]]; then
  c_ok "GTSAM $INSTALLED_GTSAM_VERSION 已安装且兼容，跳过源码编译"
else
  if [ -n "$INSTALLED_GTSAM_VERSION" ]; then
    c_warn "检测到 GTSAM $INSTALLED_GTSAM_VERSION；部署脚本将安装推荐版本 4.0.3。"
  fi
  # ---- swap 检测：ARM/Jetson 上源码编译 GTSAM 极易 OOM，务必先确认 swap ----
  _mem_kb="$(grep MemTotal /proc/meminfo | awk '{print $2}')"
  _mem_gb="$(( _mem_kb / 1024 / 1024 ))"
  _swap_kb="$(grep SwapTotal /proc/meminfo | awk '{print $2}')"
  if [ "$_mem_gb" -lt 16 ] && [ "$_swap_kb" -eq 0 ]; then
    c_err "物理内存 ${_mem_gb}GB 且无 swap：源码编译 GTSAM 极可能 OOM（尤其在 AGX Orin 上）。"
    c_warn "建议先创建 32GB swap（一次性，需 root）："
    echo "    sudo fallocate -l 32G /swapfile"
    echo "    sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile"
    echo "    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab"
    c_warn "创建后重新运行本脚本。仍将尝试继续（可能失败）。"
  elif [ "$_mem_gb" -lt 16 ]; then
    c_warn "物理内存 ${_mem_gb}GB（已检测到 swap）。若编译 OOM，请扩大 swap 至 32GB 后重试。"
  fi

  # Orin 上限制 GTSAM 编译并行度，避免 12 核全开吃满共享内存导致 OOM
  GTSAM_JOBS="$(nproc)"
  if [ "$IS_ORIN" -eq 1 ]; then
    GTSAM_JOBS=4
    c_info "AGX Orin：GTSAM 编译并行度限制为 $GTSAM_JOBS（默认全核易 OOM）。"
  fi

  c_info "源码编译安装 GTSAM 4.0.3 (约需数分钟~数十分钟，依平台而定) ..."
  GTSAM_VER="4.0.3"
  TMP="$(mktemp -d)"
  cd "$TMP"
  if [ ! -d gtsam ]; then
    git clone --depth 1 --branch "$GTSAM_VER" https://github.com/borglab/gtsam.git
  fi
  cd gtsam
  mkdir -p build && cd build
  cmake .. \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_PYTHON=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON
  make -j"$GTSAM_JOBS"
  $SUDO make install
  $SUDO ldconfig
  cd "$WS_ROOT"
  rm -rf "$TMP"
  c_ok "GTSAM 安装完成"
fi

# ---------- 3) Livox Lidar SDK（源码编译，可跳过） ----------
if [ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]; then
  c_ok "Livox Lidar SDK 已安装，跳过源码编译"
else
  c_info "源码编译安装 Livox-SDK2 ..."
  TMP="$(mktemp -d)"
  cd "$TMP"
  if [ ! -d Livox-SDK2 ]; then
    git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git
  fi
  cd Livox-SDK2
  mkdir -p build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j"$(nproc)"
  $SUDO make install
  $SUDO ldconfig
  cd "$WS_ROOT"
  rm -rf "$TMP"
  c_ok "Livox Lidar SDK 安装完成"
fi

# ---------- 4) Python 依赖 ----------
c_info "安装 Python 依赖 ..."
pip3 install --quiet --break-system-packages opencv-python numpy pyyaml 2>/dev/null \
  || pip3 install --quiet opencv-python numpy pyyaml \
  || c_warn "Python 依赖安装失败（可忽略，若运行期报缺包再手动装）"

# ---------- 5) rosdep（跳过 gtsam，因其为源码手动安装） ----------
c_info "rosdep 安装 ROS 依赖（跳过 gtsam）..."
$SUDO rosdep init 2>/dev/null || true
rosdep update 2>/dev/null || true
rosdep install --from-paths "$WS_ROOT/src" --ignore-src -y --skip-keys gtsam \
  || c_warn "rosdep 部分包未安装（多因已装或为源码包，可继续）"

# ---------- 6) Orin 专属：OpenCV 冲突自检 ----------
# JetPack 系统已带 CUDA 版 OpenCV（常 4.8/4.10），与 ros-humble 拉入的 4.5.4 并存，
# cv_bridge 链接版本若与实际运行的不一致会在运行期 ABI 崩溃。提前打印供决策。
if [ "$IS_ORIN" -eq 1 ]; then
  c_info "===== AGX Orin OpenCV 冲突自检 ====="
  _sys_cv="$(pkg-config --modversion opencv4 2>/dev/null || echo 'N/A')"
  c_info "系统 OpenCV (pkg-config opencv4): $_sys_cv"
  _cv_bridge_so="/opt/ros/${ROS_DISTRO}/lib/libcv_bridge.so"
  if [ -n "$_cv_bridge_so" ] && [ -f "$_cv_bridge_so" ]; then
    _linked_cv="$(ldd "$_cv_bridge_so" 2>/dev/null | grep -m1 'libopencv_core' | awk '{print $1}' || echo 'N/A')"
    c_info "cv_bridge 实际链接的 OpenCV: $_linked_cv"
  else
    c_info "未找到 ROS cv_bridge；仅激光测试可使用 build.sh --lidar-only。"
  fi
  c_info "两套 OpenCV 可以共存；build.sh 默认优先 cv_bridge 对应的 ROS ABI。"
fi

c_ok "依赖安装完成。下一步：bash scripts/build.sh"
