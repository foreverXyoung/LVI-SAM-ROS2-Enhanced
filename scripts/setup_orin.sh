#!/usr/bin/env bash
#
# setup_orin.sh — AGX Orin / Jetson 部署前置自检与优化
#
# 作用：
#   1) 确认 L4T / Ubuntu / ROS 2 版本（决定 Docker 标签与 OpenCV 处理）
#   2) 检查并建议 swap（源码编译 GTSAM / Ceres 极易 OOM）
#   3) 检查 OpenCV 冲突（JetPack 自带 CUDA 版 vs ros-humble 的 4.5.4）
#   4) 一键设为最大性能模式（nvpmodel -m 0 + jetson_clocks），避免限频抖动
#   5) 打印后续部署命令清单
#
# 用法：
#   bash scripts/setup_orin.sh            # 仅自检（默认，不改系统）
#   bash scripts/setup_orin.sh --apply    # 自检 + 实际创建 swap + 设性能模式
#
# 仅限 aarch64（AGX Orin / Jetson）。在 x86 上运行会直接退出并提示。
#
set -euo pipefail

c_info(){ echo -e "\033[1;34m[INFO]\033[0m  $*"; }
c_ok(){   echo -e "\033[1;32m[ OK ]\033[0m  $*"; }
c_warn(){ echo -e "\033[1;33m[WARN]\033[0m  $*"; }
c_err(){  echo -e "\033[1;31m[ERR ]\033[0m  $*"; }
c_step(){ echo -e "\033[1;35m[STEP]\033[0m  $*"; }

ARCH="$(uname -m)"
if [ "$ARCH" != "aarch64" ]; then
  c_err "本脚本仅用于 aarch64（AGX Orin / Jetson）。当前架构：$ARCH"
  c_err "x86_64 平台请直接按 docs/ENVIRONMENT.md 执行 install_deps.sh / build.sh。"
  exit 1
fi

APPLY=0
if [ "${1:-}" = "--apply" ]; then APPLY=1; fi

SUDO=""
if [ "$(id -u)" -ne 0 ]; then SUDO="sudo"; fi

echo "================================================================="
echo "  AGX Orin / Jetson 部署前置自检"
echo "================================================================="

# ---------- 1) 系统 / L4T / ROS ----------
c_step "1) 系统 / L4T / ROS 版本"
if [ -f /etc/nv_tegra_release ]; then
  L4T="$(head -n1 /etc/nv_tegra_release)"
  c_info "L4T: $L4T"
else
  c_warn "未找到 /etc/nv_tegra_release，可能不是标准 JetPack 系统。"
fi
lsb_release -ds 2>/dev/null || c_warn "lsb_release 不可用"
if [ -f /opt/ros/humble/setup.bash ]; then
  c_ok "ROS 2 Humble 已安装"
else
  c_err "未检测到 ROS 2 Humble（/opt/ros/humble）。请先安装 ros-humble-desktop。"
fi
echo

# ---------- 2) swap ----------
c_step "2) 内存 / swap"
_mem_kb="$(grep MemTotal /proc/meminfo | awk '{print $2}')"
_mem_gb="$(( _mem_kb / 1024 / 1024 ))"
_swap_kb="$(grep SwapTotal /proc/meminfo | awk '{print $2}')"
_swap_gb="$(( _swap_kb / 1024 / 1024 ))"
c_info "物理内存: ${_mem_gb} GB；当前 swap: ${_swap_gb} GB"
if [ "$_mem_gb" -lt 16 ] && [ "$_swap_kb" -eq 0 ]; then
  c_err "内存 <16GB 且无 swap：源码编译 GTSAM 几乎必 OOM！"
  c_warn "建议创建 32GB swap（--apply 会自动执行）："
  echo "    sudo fallocate -l 32G /swapfile"
  echo "    sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile"
  echo "    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab"
  if [ "$APPLY" -eq 1 ]; then
    c_info "正在创建 32GB swap ..."
    $SUDO fallocate -l 32G /swapfile
    $SUDO chmod 600 /swapfile && $SUDO mkswap /swapfile && $SUDO swapon /swapfile
    echo '/swapfile none swap sw 0 0' | $SUDO tee -a /etc/fstab
    c_ok "swap 创建并启用完成"
  fi
elif [ "$_mem_gb" -lt 16 ]; then
  c_warn "内存 <16GB 但有 swap（${_swap_gb}GB）。若编译仍 OOM，请扩大到 32GB。"
else
  c_ok "内存充足（≥16GB），可选创建 swap 以提速编译。"
fi
echo

# ---------- 3) OpenCV 冲突 ----------
c_step "3) OpenCV 冲突自检（Orin 高危坑）"
_sys_cv="$(pkg-config --modversion opencv4 2>/dev/null || echo 'N/A')"
c_info "系统 OpenCV (pkg-config opencv4): $_sys_cv"
_cv_bridge_so="/opt/ros/${ROS_DISTRO:-humble}/lib/libcv_bridge.so"
if [ -n "$_cv_bridge_so" ] && [ -f "$_cv_bridge_so" ]; then
  _linked_cv="$(ldd "$_cv_bridge_so" 2>/dev/null | grep -m1 'libopencv_core' | awk '{print $1}' || echo 'N/A')"
  c_info "cv_bridge 链接的 OpenCV: $_linked_cv"
  c_info "两套 OpenCV 可以共存；build.sh 默认优先 cv_bridge 对应的 ROS ABI。"
else
  c_info "未找到 ROS cv_bridge；仅激光测试可使用 build.sh --lidar-only。"
fi
echo

# ---------- 4) 性能模式 ----------
c_step "4) 性能模式（nvpmodel / jetson_clocks）"
if command -v nvpmodel >/dev/null 2>&1; then
  if [ "$APPLY" -eq 1 ]; then
    c_info "设置最大性能模式 ..."
    $SUDO nvpmodel -m 0 || c_warn "nvpmodel -m 0 失败（可能已在该模式）"
    $SUDO jetson_clocks || c_warn "jetson_clocks 失败"
    c_ok "已设为最大性能模式（nvpmodel -m 0 + jetson_clocks）。"
    c_info "可用 tegrastats 监控温度/利用率：sudo tegrastats --interval 2000"
  else
    c_info "当前 nvpmodel 模式：$($SUDO nvpmodel -q 2>/dev/null | head -n1 || echo '未知')"
    c_warn "未 --apply，未改动性能模式。长时间运行 SLAM 前建议执行 --apply 以限频/锁频。"
  fi
else
  c_warn "未检测到 nvpmodel（非 JetPack 标准环境），跳过性能模式设置。"
fi
echo

# ---------- 5) 后续步骤 ----------
c_step "5) 后续部署命令"
c_ok "自检完成。建议按以下顺序执行："
echo "    # a) 安装系统/ROS 依赖 + GTSAM + Livox-SDK2（已含 Orin 提示）"
echo "    bash scripts/install_deps.sh"
echo "    # b) 编译（已含 OpenCV 自动匹配 + ccache + 并行度限制）"
echo "    bash scripts/build.sh"
echo "    # c) 运行（需另起终端先启动 livox_ros_driver2 激光驱动）"
echo "    bash scripts/run.sh"
echo
c_info "更详细的步骤与话题接线验证见 docs/DEPLOY_ORIN.md。"
