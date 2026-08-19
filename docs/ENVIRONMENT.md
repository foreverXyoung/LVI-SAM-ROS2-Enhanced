# 环境配置（ENVIRONMENT）

本工程仅验证于 **Ubuntu 22.04 LTS + ROS 2 Humble Hawksbill**（其余发行版未测试）。
文档给出从零搭建可编译/可运行环境的完整步骤，并提供一键脚本与 Docker 替代方案。

---

## 1. 前置条件

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 amd64（推荐原生或 WSL2） |
| CPU | x86_64；编译 GTSAM / Ceres 较吃资源，建议 ≥ 4 核、≥ 8 GB 内存 |
| 磁盘 | 工作区 + 依赖约需 15–20 GB（GTSAM/Ceres 源码编译产物较大） |
| 网络 | 可访问 GitHub（克隆仓库与子模块、源码编译依赖） |
| 权限 | 需要 `sudo`（安装系统库、写入 `/usr/local`） |

> Windows 无法编译本工程（ROS 2 + VINS/Ceres 工具链依赖 Linux）。推荐：
> - 物理机/虚拟机装 Ubuntu 22.04；或
> - WSL2 + Ubuntu 22.04；或
> - 直接用本文末尾的 **Docker** 镜像（最省心）。

---

## 2. 安装 ROS 2 Humble（若尚未安装）

```bash
# 按官方文档安装 ros-humble-desktop（含 rclcpp/rviz 等）
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
sudo apt-get update && sudo apt-get install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get install -y ros-humble-desktop
sudo apt-get install -y ros-dev-tools   # colcon, rosdep, vcstool 等
```

验证：

```bash
source /opt/ros/humble/setup.bash
ros2 --version      # 应输出 humble
colcon version-check 2>/dev/null || echo "colcon 已安装"
```

---

## 3. 获取源码与驱动复用

```bash
git clone <your-repo-url> LVI-SAM-ROS2-Enhanced
cd LVI-SAM-ROS2-Enhanced

# 当前已 source 的工作区没有 livox_ros_driver2 时才需要：
git submodule update --init --recursive
```

`livox_ros_driver2` 可复用机器人总工作区已安装并 source 的版本；缺失时再以
**git 子模块**引入锁定版本。这样可避免重复驱动包和不必要的 GitHub 子模块下载。驱动提供 MID360 的
点云和 IMU 数据；点云为 `livox_ros_driver2/msg/CustomMsg`，IMU 必须以
`sensor_msgs/Imu` 接入。外置 IMU 使用 `imu_source:=external`（默认 `/IMU_data`）；
MID-360 内置 IMU 使用 `imu_source:=mid360`（默认 `/livox/imu`），同时加载原始 `g` 单位
转换、噪声和 IMU-LiDAR 外参。不要只覆盖 `imu_topic` 来切换物理 IMU。

---

## 4. 系统 / ROS 依赖

### 4.1 可通过 apt / rosdep 安装的部分

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git wget curl \
  libpcl-dev libopencv-dev libeigen3-dev libboost-all-dev libceres-dev \
  python3-pip python3-rosdep python3-colcon-common-extensions

# ROS 2 相关（rosdep 会自动解析 package.xml 中的 <depend>）
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys gtsam
```

> **为什么 `--skip-keys gtsam`？**
> `package.xml` 里声明了 `<depend>GTSAM</depend>`，但 GTSAM **不是 ROS 包**（无 apt 提供、
> 也不依赖 rosdep 自动选择版本）。它由已有系统安装或下面的源码步骤提供，因此 rosdep 跳过它，
> 否则 `rosdep install` 会报错退出。

### 4.2 GTSAM（复用兼容 4.x 或源码编译）

本工程使用因子图优化（ISAM2 / BetweenFactor / PriorFactor 等），需要 GTSAM 4.x。若
`GTSAMConfigVersion.cmake` 已位于 `/usr/local/lib/cmake/GTSAM`、`/usr/lib/cmake/GTSAM`
或系统多架构 CMake 路径，可直接复用；否则推荐源码构建已广泛验证的 **tag `4.0.3`**：

```bash
cd /tmp
if [ ! -d gtsam ]; then
  git clone https://github.com/borglab/gtsam.git
fi
cd gtsam
git checkout 4.0.3
mkdir -p build && cd build
cmake .. \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

验证：

```bash
ls /usr/local/lib/cmake/GTSAM        # 应存在 GTSAMConfig.cmake
pkg-config --exists gtsam && echo "gtsam ok" || echo "gtsam 未注册 pkg-config（不影响 find_package）"
```

> 备注：GTSAM 4.1 / 4.2 也可编译（API 向后兼容），但 `4.0.3` 在 LIO-SAM 生态中验证最充分。
> 若使用 GCC 11 遇到 `<tuple>` 相关编译错误，确认已加 `-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF`。

### 4.3 Livox Lidar SDK（必须源码编译，预装到 /usr/local）

`livox_ros_driver2` 的 ROS 2 分支在 CMake 中链接**系统级**的 Livox Lidar SDK
（`liblivox_lidar_sdk_shared.so` + 头文件），并不在 colcon 工作区内捆绑 SDK 源码。
因此构建驱动前必须先编译安装 Livox-SDK2：

```bash
cd /tmp
if [ ! -d Livox-SDK2 ]; then
  git clone https://github.com/Livox-SDK/Livox-SDK2.git
fi
cd Livox-SDK2
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install          # 安装到 /usr/local/lib 与 /usr/local/include
sudo ldconfig
```

验证：

```bash
ls /usr/local/lib/liblivox_lidar_sdk_shared.so   # 应存在
```

### 4.4 Python 依赖

```bash
sudo apt-get install -y python3-yaml python3-pyproj
```

视觉 C++ 节点不依赖 Python OpenCV；不要为本工程向系统 Python 额外安装
`opencv-python` 或 `numpy`。配置预检使用 PyYAML，GPS/RTK 坐标转换使用 pyproj。

---

## 5. 编译

推荐使用 [`scripts/build.sh`](../scripts/build.sh)，其等价于：

```bash
source /opt/ros/humble/setup.bash
cd <workspace-root>
git submodule update --init --recursive
colcon build --symlink-install --packages-up-to lvi_sam
source install/setup.bash
```

`--packages-up-to lvi_sam` 会**先编译 `livox_ros_driver2` 子模块**（lvi_sam 的 `<depend>`），
再编译 lvi_sam（LIS + VIS）。如只想重编本包：

```bash
colcon build --symlink-install --packages-select lvi_sam
```

编译产物：

| 可执行文件 | 子系统 |
|-----------|--------|
| `lvi_sam_imuPreintegration` | LIS（激光） |
| `lvi_sam_mapOptimization` | LIS（激光） |
| `visual_feature_node` | VIS（视觉） |
| `visual_estimator_node` | VIS（视觉） |
| `visual_loop_node` | VIS（视觉） |

---

## 6. 版本核验清单

部署完成后逐项确认：

```bash
# ROS 2
ros2 --version                                  # humble

# GTSAM
ls /usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake # 存在

# Livox SDK
ldconfig -p | grep livox_lidar_sdk              # 有 liblivox_lidar_sdk_shared.so

# 子模块
git submodule status                            # livox_ros_driver2 前有空格（已锁定 1.1.1），无 '+'/'-'

# 工作区编译
source install/setup.bash
ros2 pkg list | grep -E "lvi_sam|livox_ros_driver2"   # 两者都应列出
```

---

## 7. Docker 替代方案（推荐用于 CI / 干净部署）

如果不想在宿主机装一堆系统库，直接用仓库根目录的 [`Dockerfile`](../Dockerfile)：

```bash
# 宿主机先初始化子模块（Docker 构建上下文需要它）
git submodule update --init --recursive

# 构建镜像（镜像内会自动源码编译 GTSAM / Livox-SDK2 并 colcon build）
docker build -t lvi-sam-ros2-enhanced .

# 运行（如需连接真实 MID360，挂载 USB 设备；仿真则不需要）
docker run -it --rm --net=host \
  -v /dev:/dev --privileged \
  lvi-sam-ros2-enhanced bash
```

`docker-compose.yml` 提供更省心的默认（`--net=host` + 设备挂载 + 工作目录）。
详见 [`docker-compose.yml`](../docker-compose.yml) 注释。

---

## 8. 常见环境坑

| 现象 | 原因 | 解决 |
|------|------|------|
| `rosdep install` 报 `gtsam` 无法解析 | GTSAM 非 ROS 包 | 加 `--skip-keys gtsam`，并确认已按 §4.2 源码安装 |
| `Could NOT find GTSAM` | GTSAM 未安装或未 `ldconfig` | 重跑 §4.2，`sudo ldconfig` |
| `liblivox_lidar_sdk_shared.so: cannot open` | Livox-SDK2 未装 | 重跑 §4.3，`sudo ldconfig` |
| `livox_ros_driver2` 编译报 `custom_msg` 找不到 | 子模块未初始化 | `git submodule update --init --recursive` |
| `colcon` 报 `fairino_description` 缺失 | 旧 package.xml 残留平台依赖 | 已在本版删除；`git pull` 最新后重编 |
| Eigen3 找不到 `/opt/eigen` | 上游 LVI-SAM-ROS2 硬编码 | 本工程已改用 `find_package(Eigen3)`，无需处理 |
