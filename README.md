# LVI-SAM-ROS2-Enhanced

> Lidar-Visual-Inertial SLAM for ROS 2 — **话题级松耦合**架构：
> **LIS（激光惯性里程计，源自 ROS 2 版 LIO-SAM 改进 fork）** + **VIS（视觉惯性里程计，源自 LVI-SAM-ROS2）**。

本工程在「思诺摘钩机器人」项目原有跑通的 ROS 2 LIO-SAM 定位模块基础上，按
[LVI-SAM](https://github.com/TixiaoShan/LVI-SAM) 的架构组织方式重构，并把**视觉子系统（VIS）**
以独立、可开关的节点栈形式接入，实现激光/视觉双模态定位。两子系统通过 ROS 话题交换信息，
而非把相机重投影因子塞进激光因子图（即「话题级松耦合」，详见下文架构）。

## 0. 当前版本与阅读顺序

当前维护和默认部署分支为 `main`。增强功能合并锚点为 `f140df2`，其中完整增强实现来自
`19ca76a`，并保留旧 `main` 的 ROS 环境脚本兼容修复。`agent/visual-rviz-defaults` 仅作为
合并前历史备份；目标机部署后仍应执行 `git rev-parse HEAD` 和 `git status --short`，记录
实际检出的版本和本地改动。

建议按以下顺序阅读：

1. [`docs/TECHNICAL_IMPROVEMENT_REPORT_ZH.md`](docs/TECHNICAL_IMPROVEMENT_REPORT_ZH.md)：面向第三方专家的正式技术改进说明；
2. [`docs/CHANGE_SUMMARY.md`](docs/CHANGE_SUMMARY.md)：内部改动记录、兼容性结论和排障证据；
3. [`src/lvi_sam/config/README.md`](src/lvi_sam/config/README.md)：场景、IMU、安装外参和相机配置入口；
4. [`docs/USAGE.md`](docs/USAGE.md)：日常建图/定位/视觉启动流程；
5. [`docs/LOCALIZATION_STATUS_INTERFACE.md`](docs/LOCALIZATION_STATUS_INTERFACE.md) 与
   [`docs/LOCALIZATION_RESET_INTERFACE.md`](docs/LOCALIZATION_RESET_INTERFACE.md)：上层状态机和复位事件契约；
6. [`docs/LOCALIZATION_ACCEPTANCE_MATRIX.md`](docs/LOCALIZATION_ACCEPTANCE_MATRIX.md)：状态输出、事件边界和原算法不受干预的实机验收顺序。

当前文档不会把“能够编译”写成“已经完成实机验收”。完整 ROS 2、GTSAM、PCL、Ceres 构建和
真实传感器运行仍以 Orin 的命令输出为准。

---

## 1. 特性

- **LIS（激光惯性 SLAM）**：源自 ROS 2 版 LIO-SAM 改进 fork，已集成
  MID360 激光前端、RTK/GNSS 因子、ScanContext 重定位、先验地图加载（LoadPriorMap）、
  丢失检测与 `force_relocalize` 服务等。
- **VIS（视觉惯性 SLAM）**：源自 [LVI-SAM-ROS2](https://github.com/Rudra2302/LVI-SAM-ROS2)，
  含 feature_tracker / estimator / visual_loop 三节点，使用 DBoW2 词袋回环。
- **双向耦合（三个接口）**：
  1. **LIS → VIS**：`/odometry/imu` 可作为 VIS 位姿、速度和尺度初始化先验；由
     `use_lidar_odometry_prior` 独立控制，完成 LiDAR-camera 标定后再启用。
  2. **LIS → VIS**：`/lio_sam/deskew/cloud_deskewed` 激光去畸变点云，为视觉提供深度/时间同步参考。
  3. **VIS → LIS**：`/lvi_sam/vins/loop/match_frame`（Float64MultiArray 时间戳对）作为**视觉回环候选**，
     接入 LIS 已有的外部回环节点（`lio_loop/loop_closure_detection`），与 ScanContext 回环并列交叉验证。
- `/odometry/imu` 保留原 LVI-SAM 的 VIS 初始化元数据兼容契约（重置编号、IMU bias、重力）。
  这些字段不是统计协方差；准确索引、校验规则及面向 Nav2 的使用边界见
  [`docs/INTERFACES_AND_STABILITY.md`](docs/INTERFACES_AND_STABILITY.md#321-lisvis-内部里程计元数据)。
- **定位状态接口**：上层状态机使用 `/lio_sam/localization/status` 的结构化
  `LocalizationStatus`；`/lio_sam/localization/reset` 是独立的观察性事件。当前内部估计器
  不订阅该事件，状态接口不会清空 IMU/VIS 队列、重建因子图或修改 bias。
  验收顺序见 [`docs/LOCALIZATION_ACCEPTANCE_MATRIX.md`](docs/LOCALIZATION_ACCEPTANCE_MATRIX.md)。
- **配置外置**：全部参数集中在 [`src/lvi_sam/config/`](src/lvi_sam/config/README.md)，
  随包安装（`install(DIRECTORY config)`），无源码硬编码路径。
- **源码二分**：`src/lvi_sam/src/` 下严格分为 `lidar_odometry/`（激光）与 `visual_odometry/`（视觉），
  参考 LVI-SAM 原版布局。

---

## 2. 仓库结构

```
LVI-SAM-ROS2-Enhanced/                 # 可独立作为工作区，也可放入现有 <workspace>/src
├── .gitignore                         # 忽略 build/install/log 与 *.pcd/*.scd/*.log 等大数据
├── .gitmodules                        # livox_ros_driver2 子模块声明
├── LICENSE                            # BSD-3-Clause（lvi_sam 主体）
├── README.md
└── src/
    ├── lvi_sam/                       # 本工程主包（LIS + VIS）
    │   ├── CMakeLists.txt             # 5 个 executable：LIS 2 + VIS 3
    │   ├── package.xml
    │   ├── config/                    # 统一配置入口（场景/模式 YAML、相机、RViz、词表）
    │   ├── launch/
    │   │   └── run.launch.py          # 总入口：LIS/VIS/RViz，可选 robot_state_publisher
    │   ├── scripts/                   # gps_to_cartesian_node.py 等辅助脚本
    │   ├── include/                   # utility.hpp / sc（ScanContext）等
    │   └── src/
    │       ├── lidar_odometry/        # 激光 LIS：imuPreintegration.cpp / mapOptmization.cpp / *.hpp
    │       └── visual_odometry/       # 视觉 VIS：visual_feature / visual_estimator / visual_loop
    └── livox_ros_driver2/             # ⚠️ git 子模块（MIT），见第 3 节
```

> **为什么 livox_ros_driver2 用子模块而非复制源码？**
> 它是第三方驱动（MIT 许可），以子模块引用可保持本仓库轻量、可追溯上游版本，
> 符合开源协作惯例。若机器人工作区没有已安装驱动，再使用 `--recursive` 克隆或执行
> `git submodule update --init`；已有并已 source 驱动时无需下载第二份。

---

## 3. 依赖

### 3.1 系统 / ROS 依赖

| 类别 | 内容 |
|------|------|
| ROS 2 | **Humble Hawksbill**（其余发行版未验证） |
| Livox Lidar SDK | 仅在本机需要重新编译 `livox_ros_driver2` 时安装到 `/usr/local`；直接复用机器人工作区中已安装的驱动消息接口时无需重复安装。 |
| apt 包 | `ros-humble-desktop`（或至少 `ros-humble-rclcpp`）、`ros-humble-pcl-*`、`ros-humble-tf2*`、`ros-humble-robot-state-publisher`、`ros-humble-xacro`；以及 `libpcl-dev`、`libopencv-dev`、`libeigen3-dev`、`libboost-all-dev`、`libceres-dev` 等 |
| 源码编译 | **GTSAM**（无兼容版本时源码构建，详见 [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md)）；Livox Lidar SDK 仅在重编驱动时需要 |
| Python | apt/rosdep 包 `python3-yaml`、`python3-pyproj`；以及 `python3-rosdep`、`python3-colcon-common-extensions` |

> ⚠️ Ubuntu 22.04 官方源通常不提供本工程所需的 GTSAM 开发包；已有兼容 GTSAM 4.x 时可直接复用，
> 否则从源码构建推荐 tag `4.0.3`。本仓库的 [`scripts/install_deps.sh`](scripts/install_deps.sh)
> 会先检查 `/usr/local` 和系统 CMake 路径，再按需完成 GTSAM 与 Livox-SDK2 的源码编译安装，
> 并对 `rosdep` 跳过 `gtsam` 键（因为它由脚本手动安装，非 ROS 包）。

### 3.2 工作区依赖（子模块）

- **livox_ros_driver2**（`src/livox_ros_driver2`，pin 到上游 **v1.1.1**）：提供 MID360 的
  `/livox/lidar` CustomMsg 点云；机器人 IMU 驱动需在 `/IMU_data` 发布标准 `sensor_msgs/Imu`。

### 3.3 平台集成要求（需自备，不随本仓库发布）

- **机器人 URDF / xacro**：核心估计器和新安装 profile 不从 TF tree 读取外参；若 Nav2/RViz
  需要完整机器人模型，仍应提供 `base_link → livox_frame → imu_link`（以及相机 `camera*` 帧）
  的静态链。`run.launch.py` 可通过 `robot_description_file` 启动 `robot_state_publisher`。
- **相机驱动**：VIS 默认订阅 `/camera/color/image_raw`（sensor_msgs/Image）；可通过
  `image_topic` 启动参数覆盖。相机模型、分辨率、内参与外参必须使用实机标定值。
- **IMU 选择**：LIS 与 VIS 默认使用 `imu_source:=external`（`/IMU_data`）；测试 MID-360
  内置 IMU 使用 `imu_source:=mid360`（`/livox/imu`）。两套 profile 同时切换话题、单位、
  噪声与 IMU-LiDAR 外参，不能只用 `imu_topic` 更换物理 IMU。
- **相机 profile**：`camera_params_file` 留空时随 `imu_source` 自动选择；MID-360 文件已经写入
  实机 `T_cam_radar` 并启用激光深度。相机—IMU 平移仍包含 FAST-LIO 的通用内置杆臂初值，
  所以 LIS 里程计先验和全局视觉—激光对齐继续分阶段启用。

所有传感器安装外参只存在于 `src/lvi_sam/config/` 的 profile 中。源码仅实现坐标约定、参数
校验和 SE(3) 组合；缺少所需外参时启动失败，不从 TF、单位矩阵或零平移推测实机安装关系。

---

## 4. 构建

**推荐（一键部署）**：见 [`scripts/`](scripts/) 与 [`docs/ENVIRONMENT.md`](docs/ENVIRONMENT.md)。

```bash
# 0) 克隆本仓库。已有机器人工作区驱动时无需重复拉取子模块。
git clone <your-fork-url> LVI-SAM-ROS2-Enhanced
cd LVI-SAM-ROS2-Enhanced

# 仅在当前已 source 的工作区中找不到 livox_ros_driver2 时执行：
# git submodule update --init --recursive

# 1) 一键完成：子模块 → 系统依赖 → 复用或构建 GTSAM/Livox-SDK2 → rosdep → colcon 编译
bash scripts/setup.sh

# 或分步：
#   bash scripts/install_deps.sh   # apt + 复用/构建 GTSAM + Livox-SDK2 + rosdep
#   bash scripts/build.sh          # 全量构建 LIS + VIS
#   bash scripts/build.sh --lidar-only --clean  # 首轮上车：只构建 LIS，缩短编译与排障链路
#   python3 src/lvi_sam/scripts/validate_config.py --config-dir src/lvi_sam/config
#   python3 src/lvi_sam/scripts/validate_config.py --config-dir src/lvi_sam/config --lidar-only
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

若仓库位于 `/data/return_station_ws/src/LVI-SAM-ROS2-Enhanced`，`build.sh` 和
`run.sh` 会自动使用 `/data/return_station_ws` 的 `build/install`，不会再产生嵌套工作区。
可用 `LVI_SAM_WORKSPACE_ROOT=/path/to/ws` 显式覆盖自动检测。

VIS 使用工程内的 `sensor_msgs/Image ↔ cv::Mat` 适配层，不直接链接 ROS 预编译
`cv_bridge`。因此 JetPack OpenCV 4.8 与 ROS Humble OpenCV 4.5 可以保留在系统中，
LVI-SAM 每个进程只会加载 CMake 选中的一套 OpenCV。迁移到新机器后正常重新编译即可；
只有在机器上存在多个 `OpenCVConfig.cmake` 且默认选择不符合预期时，才需要显式设置
`OpenCV_DIR`。详见 [`docs/DEPLOY_ORIN.md`](docs/DEPLOY_ORIN.md#3-opencv-单一依赖策略)。

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
| `/lio_sam/mapping/cloud_registered_raw` | `sensor_msgs/PointCloud2` | 当前去畸变扫描注册到 `odometryFrame`，供后续深度/配准流程 |
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
| [`docs/TECHNICAL_IMPROVEMENT_REPORT_ZH.md`](docs/TECHNICAL_IMPROVEMENT_REPORT_ZH.md) | **第三方专家技术报告**：比较基线、总体架构、关键改进、算法影响边界、接口、验证证据和待量化风险。 |
| [`src/lvi_sam/config/README.md`](src/lvi_sam/config/README.md) | **统一配置入口**：场景/模式选择矩阵、参数覆盖优先级、标定分组、接口清单和修改后预检。 |
| [`docs/CHANGE_SUMMARY.md`](docs/CHANGE_SUMMARY.md) | **完整改动与复审记录**：模块级改动、原逻辑兼容性、有意变化、已修问题、验证状态和已知边界。 |
| [`docs/ENVIRONMENT.md`](docs/ENVIRONMENT.md) | **详细环境配置**：Ubuntu 22.04 + ROS 2 Humble、apt 依赖、GTSAM 源码编译、Livox-SDK2 安装、子模块、rosdep、Docker 替代方案、版本核验。 |
| [`docs/USAGE.md`](docs/USAGE.md) | **详细使用说明**：构建、URDF/相机/IMU 准备、launch 参数、话题接线表、参数文件、最小验证闭环、排错与调参。 |
| [`docs/ARCHITECTURE_AND_MAP_FORMAT.md`](docs/ARCHITECTURE_AND_MAP_FORMAT.md) | 模块职责、地图版本、视觉跨会话重定位数据集和 RTK 输入契约。 |
| [`docs/INTERFACES_AND_STABILITY.md`](docs/INTERFACES_AND_STABILITY.md) | **工程接口与稳定性契约**：模块边界、话题/QoS、时间与坐标、依赖所有权、降级策略、扩展规则和 Orin 验收门槛。 |
| [`docs/LOCALIZATION_STATUS_INTERFACE.md`](docs/LOCALIZATION_STATUS_INTERFACE.md) / [`docs/LOCALIZATION_RESET_INTERFACE.md`](docs/LOCALIZATION_RESET_INTERFACE.md) | 定位状态位、复位事件字段、代次语义和上层控制边界。 |
| [`docs/LOCALIZATION_ACCEPTANCE_MATRIX.md`](docs/LOCALIZATION_ACCEPTANCE_MATRIX.md) | **定位接口验收矩阵**：验证状态、事件、IMU/VINS 数据链以及状态输出不干预原算法。 |
| [`docs/REMOTE_TEST_AND_CHANGES.md`](docs/REMOTE_TEST_AND_CHANGES.md) | **本次修改说明与远程测试手册**：拉取、编译、分阶段建图/定位、视觉、RTK、验收及已知边界。 |
| [`scripts/setup.sh`](scripts/setup.sh) | 一键编排：子模块初始化 → 依赖安装 → 编译。 |
| [`scripts/install_deps.sh`](scripts/install_deps.sh) | 安装系统/ROS 依赖，复用兼容 GTSAM 4.x 或按需源码构建 GTSAM/Livox-SDK2（重跑安全）。 |
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
