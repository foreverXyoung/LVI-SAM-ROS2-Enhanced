# LVI-SAM-ROS2-Enhanced

> Lidar-Visual-Inertial SLAM for ROS 2 — **话题级松耦合**架构：
> **LIS（激光惯性里程计，源自 ROS 2 版 LIO-SAM 改进 fork）** + **VIS（视觉惯性里程计，源自 LVI-SAM-ROS2）**。

本工程在「思诺摘钩机器人」项目原有跑通的 ROS 2 LIO-SAM 定位模块基础上，按
[LVI-SAM](https://github.com/TixiaoShan/LVI-SAM) 的架构组织方式重构，并把**视觉子系统（VIS）**
以独立、可开关的节点栈形式接入，实现激光/视觉双模态定位。两子系统通过 ROS 话题交换信息，
而非把相机重投影因子塞进激光因子图（即「话题级松耦合」，详见下文架构）。

---

## 1. 特性

- **LIS（激光惯性 SLAM）**：源自 ROS 2 版 LIO-SAM 改进 fork，已集成
  MID360 激光前端、RTK/GNSS 因子、ScanContext 重定位、先验地图加载（LoadPriorMap）、
  丢失检测与 `force_relocalize` 服务等。
- **VIS（视觉惯性 SLAM）**：源自 [LVI-SAM-ROS2](https://github.com/Rudra2302/LVI-SAM-ROS2)，
  含 feature_tracker / estimator / visual_loop 三节点，使用 DBoW2 词袋回环。
- **双向耦合（三个接口）**：
  1. **LIS → VIS**：`/lio_sam/odometry/imu` 作为 VIS 位姿 + **尺度**先验（VIS 为单目，离 LIS 会尺度失控）。
  2. **LIS → VIS**：`/lio_sam/deskew/cloud_deskewed` 激光去畸变点云，为视觉提供深度/时间同步参考。
  3. **VIS → LIS**：`/lvi_sam/vins/loop/match_frame`（Float64MultiArray 时间戳对）作为**视觉回环候选**，
     接入 LIS 已有的外部回环节点（`lio_loop/loop_closure_detection`），与 ScanContext 回环并列交叉验证。
- **配置外置**：全部参数集中在 `src/lvi_sam/config/`，随包安装（`install(DIRECTORY config)`），
  无源码硬编码路径。
- **源码二分**：`src/lvi_sam/src/` 下严格分为 `lidar_odometry/`（激光）与 `visual_odometry/`（视觉），
  参考 LVI-SAM 原版布局。

---

## 2. 仓库结构

```
LVI-SAM-ROS2-Enhanced/                 # colcon 工作区根（即本仓库根）
├── .gitignore                         # 忽略 build/install/log 与 *.pcd/*.scd/*.log 等大数据
├── .gitmodules                        # livox_ros_driver2 子模块声明
├── LICENSE                            # BSD-3-Clause（lvi_sam 主体）
├── README.md
└── src/
    ├── lvi_sam/                       # 本工程主包（LIS + VIS）
    │   ├── CMakeLists.txt             # 5 个 executable：LIS 2 + VIS 3
    │   ├── package.xml
    │   ├── config/                    # 外层集中配置（params_lidar*.yaml / params_camera.yaml / 词表）
    │   ├── launch/
    │   │   └── run.launch.py          # 总入口：启动 7 节点 + 话题 remap 接线
    │   ├── scripts/                   # gps_to_cartesian_node.py 等辅助脚本
    │   ├── include/                   # utility.hpp / sc（ScanContext）等
    │   └── src/
    │       ├── lidar_odometry/        # 激光 LIS：imuPreintegration.cpp / mapOptmization.cpp / *.hpp
    │       └── visual_odometry/       # 视觉 VIS：visual_feature / visual_estimator / visual_loop
    └── livox_ros_driver2/             # ⚠️ git 子模块（MIT），见第 3 节
```

> **为什么 livox_ros_driver2 用子模块而非复制源码？**
> 它是第三方驱动（MIT 许可），以子模块引用可保持本仓库轻量、可追溯上游版本，
> 符合开源协作惯例。远程仓库 clone 时需 `--recursive` 或在本地执行
> `git submodule update --init`。

---

## 3. 依赖

### 3.1 系统 / ROS 依赖

| 类别 | 内容 |
|------|------|
| ROS 2 | **Humble Hawksbill**（其余发行版未验证） |
| Livox Lidar SDK | **必须预装到 `/usr/local`**：克隆 [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)，`cmake -> make -> sudo make install`。`livox_ros_driver2` 的 CMake 通过 `/usr/local/lib` 与头文件查找它。 |
| apt 包 | `ros-humble-desktop`（或至少 `ros-humble-rclcpp`）、`ros-humble-pcl-*`、`ros-humble-tf2*`、`ros-humble-robot-state-publisher`、`ros-humble-xacro`；以及 `libpcl-dev`、`libopencv-dev`、`libeigen3-dev`、`libboost-all-dev`、`libceres-dev` 等 |
| 源码编译 | **GTSAM（无 apt 包，须源码编译，详见 [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md)）**；**Livox Lidar SDK（须预装到 `/usr/local`）** |
| Python | `opencv-python`、`numpy`、`pyyaml`（VIS / 脚本使用）；以及 `python3-rosdep`、`python3-colcon-common-extensions` |

> ⚠️ **Ubuntu 22.04 / ROS 2 Humble 下没有 `libgtsam-dev` apt 包**，GTSAM 必须从源码编译（推荐 tag `4.0.3`）。
> 本仓库提供的 [`scripts/install_deps.sh`](scripts/install_deps.sh) 会自动完成 GTSAM 与 Livox-SDK2 的源码编译安装，
> 并对 `rosdep` 跳过 `gtsam` 键（因为它由脚本手动安装，非 ROS 包）。

### 3.2 工作区依赖（子模块）

- **livox_ros_driver2**（`src/livox_ros_driver2`，pin 到上游 **v1.1.1**）：提供 MID360 的
  `/livox/lidar` CustomMsg 点云；机器人 IMU 驱动需在 `/IMU_data` 发布标准 `sensor_msgs/Imu`。

### 3.3 平台集成要求（需自备，不随本仓库发布）

- **机器人 URDF / xacro**：必须提供 `base_link → livox_frame → imu_link`（以及相机 `camera*` 帧）
  的 TF 链。`run.launch.py` 通过 `robot_description_file` 启动参数接收，并可选启动
  `robot_state_publisher`。请使用你机器人的真实外参替换。
- **相机驱动**：VIS 默认订阅 `/camera/color/image_raw`（sensor_msgs/Image）；可通过
  `image_topic` 启动参数覆盖。相机模型、分辨率、内参与外参必须使用实机标定值。
- **IMU 话题**：LIS 与 VIS 默认共用 `/IMU_data`，类型必须是 `sensor_msgs/Imu`。
  若现场驱动使用其他话题，通过入口 launch 的 `imu_topic` 一次性覆盖 LIS 与 VIS。

---

## 4. 构建

**推荐（一键部署）**：见 [`scripts/`](scripts/) 与 [`docs/ENVIRONMENT.md`](docs/ENVIRONMENT.md)。

```bash
# 0) 克隆本仓库（含子模块）
git clone --recursive <your-fork-url> LVI-SAM-ROS2-Enhanced
cd LVI-SAM-ROS2-Enhanced

# 若已 clone 但未带子模块：
# git submodule update --init --recursive

# 1) 一键完成：子模块 → 系统依赖 → GTSAM/Livox-SDK2 源码编译 → rosdep → colcon 编译
bash scripts/setup.sh

# 或分步：
#   bash scripts/install_deps.sh   # apt + GTSAM(源码) + Livox-SDK2(源码) + rosdep
#   bash scripts/build.sh          # 全量构建 LIS + VIS
#   bash scripts/build.sh --lidar-only --clean  # 首轮上车：只构建 LIS，隔离相机/OpenCV
#   bash scripts/run.sh            # 启动 LIS + VIS
```

手动构建等价于上面的脚本化流程：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y --skip-keys gtsam
colcon build --symlink-install --packages-up-to lvi_sam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_VISUAL=ON
source install/setup.bash
```

---

## 5. 运行

```bash
# 启动激光驱动（livox_ros_driver2，自带 MID360 launch）
ros2 launch livox_ros_driver2 msg_MID360_launch.py   # 或你的平台对应 launch

# 启动 LIS + VIS + RViz2（视觉与 RViz 默认开启，均可独立关闭）
ros2 launch lvi_sam run.launch.py \
    robot_description_file:=/path/to/your_robot.urdf.xacro
```

无图形界面的远程终端使用 `enable_rviz:=false`；仅测试激光链路时再加
`enable_visual:=false`。

主要话题（节选）：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | MID360 原始点云（LIS 输入） |
| `/IMU_data` | `sensor_msgs/Imu` | LIS 与 VIS 共用的标定后 IMU |
| `/lio_sam/mapping/odometry` | `nav_msgs/Odometry` | LIS 里程计 → 对外 |
| `/odometry/imu` | `nav_msgs/Odometry` | **LIS → VIS 位姿+尺度先验** |
| `/lio_sam/deskew/cloud_deskewed` | `sensor_msgs/PointCloud2` | **LIS → VIS 激光深度** |
| `/lvi_sam/vins/loop/match_frame` | `std_msgs/Float64MultiArray` | **VIS → LIS 视觉回环候选** |
| `/lio_sam/mapping/cloud_registered` | `sensor_msgs/PointCloud2` | LIS 建图点云 |

---

## 6. 已知限制 / Todo

- [ ] 相机–IMU–激光外参标定与启动期时间同步。
- [ ] 视觉 DBoW2/BRIEF 数据库目前只存在于运行内存；跨会话重定位仍由
  Scan Context + ICP 完成，视觉地图持久化格式见架构文档。
- [ ] RTK 核心接收地图对齐的 `nav_msgs/Odometry`；原始 `NavSatFix` 的转换、
  RTK 解状态判定与天线杆臂补偿由上游转换节点负责。
- [ ] 站场高度对称/长直股道下 ScanContext 误闭环风险 → 需加 RTK/先验地图一致性门控后再接受回环。
- [ ] VIS → LIS 前端初值猜测（`imu_propagate_ros`，耦合点③a）暂未接入，最小验证闭环先不做。

---

## 7. 文档与部署脚本

| 文件 | 用途 |
|------|------|
| [`docs/ENVIRONMENT.md`](docs/ENVIRONMENT.md) | **详细环境配置**：Ubuntu 22.04 + ROS 2 Humble、apt 依赖、GTSAM 源码编译、Livox-SDK2 安装、子模块、rosdep、Docker 替代方案、版本核验。 |
| [`docs/USAGE.md`](docs/USAGE.md) | **详细使用说明**：构建、URDF/相机/IMU 准备、launch 参数、话题接线表、参数文件、最小验证闭环、排错与调参。 |
| [`docs/ARCHITECTURE_AND_MAP_FORMAT.md`](docs/ARCHITECTURE_AND_MAP_FORMAT.md) | 模块职责、地图版本、视觉跨会话重定位数据集和 RTK 输入契约。 |
| [`docs/REMOTE_TEST_AND_CHANGES.md`](docs/REMOTE_TEST_AND_CHANGES.md) | **本次修改说明与远程测试手册**：拉取、编译、分阶段建图/定位、视觉、RTK、验收及已知边界。 |
| [`scripts/setup.sh`](scripts/setup.sh) | 一键编排：子模块初始化 → 依赖安装 → 编译。 |
| [`scripts/install_deps.sh`](scripts/install_deps.sh) | 安装系统/ROS 依赖，并源码编译安装 GTSAM 与 Livox-SDK2（重跑安全）。 |
| [`scripts/build.sh`](scripts/build.sh) | `colcon build --symlink-install --packages-up-to lvi_sam`；优先复用已 source 的 `livox_ros_driver2`，否则初始化并编译子模块。 |
| [`scripts/run.sh`](scripts/run.sh) | source 环境并 `ros2 launch lvi_sam run.launch.py`，支持透传参数。 |
| [`Dockerfile`](Dockerfile) / [`docker-compose.yml`](docker-compose.yml) | 可复现的容器化环境（ros:humble + 全部依赖 + 编译）。 |

---

## 8. 许可证与致谢

- **本工程主体 `lvi_sam`**：BSD-3-Clause（继承自 LIO-SAM / LVI-SAM）。
- **`livox_ros_driver2`（子模块）**：MIT，© Livox Tech。
- **VIS 内 `visual_loop/ThirdParty/`（DBoW2 / DVision 等）**：各自保留原作者许可，
  请勿移除其 LICENSE / 版权声明。

上游工作（致敬）：

- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) — Tixiao Shan 等（激光惯性 SLAM 因子图）。
- [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM) — 同上作者（激光-视觉-惯性联合 SLAM）。
- [LVI-SAM-ROS2](https://github.com/Rudra2302/LVI-SAM-ROS2) — VIS 节点的 ROS 2 移植。
- 本项目所用 `lio_sam` 改进 fork 在官方 LIO-SAM 之上增加了 MID360 前端、RTK、ScanContext 重定位、
  先验地图等（详见工程内分析文档）。

> 本仓库仅包含算法工程的源码与配置；**不**包含任何建图产物（`*.pcd` / `*.scd`）、运行日志或数据集，
> 这些均被 `.gitignore` 排除。
