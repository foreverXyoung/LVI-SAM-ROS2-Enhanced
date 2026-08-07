#!/usr/bin/env bash
#
# setup.sh — 一键部署 LVI-SAM-ROS2-Enhanced
#
#   1) 初始化 git 子模块（livox_ros_driver2）
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

# 1) 子模块
echo "[1/3] 初始化子模块 ..."
git submodule update --init --recursive

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
