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

# 终端颜色
c_info(){ echo -e "\033[1;34m[INFO]\033[0m  $*"; }
c_ok(){   echo -e "\033[1;32m[ OK ]\033[0m  $*"; }
c_warn(){ echo -e "\033[1;33m[WARN]\033[0m  $*"; }
c_err(){  echo -e "\033[1;31m[ERR ]\033[0m  $*"; }

# ---------- 0) 检测 ROS 2 环境 ----------
c_info "检测 ROS 2 ($ROS_DISTRO) 环境 ..."
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  c_ok "已 source /opt/ros/$ROS_DISTRO/setup.bash"
elif [ -n "${ROS_DISTRO:-}" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/$ROS_DISTRO/setup.bash"
else
  c_err "未找到 ROS 2 ($ROS_DISTRO)。请先按 docs/ENVIRONMENT.md 安装 ROS 2 Humble。"
  exit 1
fi

# ---------- 1) apt 基础包 ----------
c_info "安装 apt / 编译基础包 ..."
$SUDO apt-get update -y
$SUDO apt-get install -y --no-install-recommends \
  build-essential cmake git wget curl ca-certificates \
  libpcl-dev libopencv-dev libeigen3-dev libboost-all-dev libceres-dev \
  python3-pip python3-rosdep python3-colcon-common-extensions \
  ros-"$ROS_DISTRO"-desktop ros-"$ROS_DISTRO"-pcl-ros \
  ros-"$ROS_DISTRO"-pcl-conversions ros-"$ROS_DISTRO"-tf2* \
  ros-"$ROS_DISTRO"-robot-state-publisher ros-"$ROS_DISTRO"-xacro

# ---------- 2) GTSAM（源码编译，可跳过） ----------
if [ -f /usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake ]; then
  c_ok "GTSAM 已安装，跳过源码编译"
else
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
  make -j"$(nproc)"
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

c_ok "依赖安装完成。下一步：bash scripts/build.sh"
