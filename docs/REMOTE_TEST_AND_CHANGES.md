# 远程部署、测试流程与本次修改说明

本文面向 Ubuntu 22.04 / ROS 2 Humble 实机，给出从拉取代码、编译、建图、保存地图、加载先验地图定位，到视觉与 RTK 验证的完整流程。建议严格按顺序执行，先验证纯激光链路，再逐项启用视觉和 RTK，避免多个传感器问题叠加后难以定位原因。

## 1. 本次修改解决了什么

### 1.1 建图与先验地图重定位

- 将 `mapping` 与 `localization` 配置明确分离，并由 `mode`、`scene` 两个 launch 参数选择。
- 建图结束后保存轨迹、关键帧位姿、原始扫描、角点/面点和 Scan Context 描述子。
- 定位模式启动时校验地图文件、关键帧数量、Scan Context 维度及坐标帧。
- 新地图增加 `map_manifest.yaml`；旧地图没有清单时仍按兼容模式加载。
- 支持 Scan Context 全局候选、ICP 几何验证、定位丢失检测以及 `/lio_sam/localization/force_relocalize` 强制重定位服务。

### 1.2 回环检测

- 保留原视觉 DBoW2 + BRIEF 回环链路。
- 激光侧增加可配置的在线 Scan Context 回环，候选最终必须通过 ICP 后才加入 GTSAM 位姿图。
- 修复回环线程访问关键帧数据时的并发快照、下标检查和候选范围问题。
- 视觉回环修复相机模型未初始化、队列无上限、异常图像/特征输入和关键帧内存生命周期问题。
- 修复相机—IMU 外参及 `td`/滚动快门时间参数的 ROS 2 类型契约；时间偏移保留浮点精度，并在启动时拒绝非法旋转矩阵。

### 1.3 ROS 2 与 MID360

- 工程包、CMake、安装规则和依赖统一为 ROS 2 Humble 的 `lvi_sam` 包。
- launch 支持参数文件自动选择、统一 IMU/RTK 话题、可关闭视觉节点、可选 URDF/Xacro 和仿真时间。
- MID360 使用 `livox_ros_driver2/msg/CustomMsg`，并保留原项目已经验证过的 MID360 前端逻辑。
- 加入传感器类型、扫描规模、量程、体素、IMU 噪声、外参矩阵和 TF 新鲜度的启动期校验，错误配置会尽早失败并给出原因。

### 1.4 RTK 融合

- 建图 `useGpsFactor` 与定位 `Loc.useRTKAssist` 分开控制，默认不隐式启用全局因子。
- 输入统一为已经与 LIO 地图对齐的 `nav_msgs/Odometry`，并检查时间、`frame_id`、有限值和协方差。
- 建图 GPS 因子增加距离/创新量门限、方差下限和 Huber robust kernel。
- 定位初始化要求多帧稳定 RTK；在线辅助使用绝对创新量、归一化创新量及自适应融合权重。
- 可选双天线航向，但只有四元数和 yaw 协方差都可信时才接收。

## 2. 远程电脑首次部署

### 2.1 拉取代码

```bash
cd ~/work
git clone --recursive https://github.com/foreverXyoung/LVI-SAM-ROS2-Enhanced.git
cd LVI-SAM-ROS2-Enhanced

# 已经克隆过时使用：
git pull --ff-only origin main
git submodule update --init --recursive
```

记录当前测试版本，便于复现：

```bash
git rev-parse HEAD
git status --short
```

第二条命令在正式测试前应没有输出。

### 2.2 安装依赖并编译

```bash
source /opt/ros/humble/setup.bash
bash scripts/install_deps.sh
# 第一阶段只验证 MID-360 + IMU + Scan Context + 建图/重定位
bash scripts/build.sh --lidar-only --clean
source install/setup.bash
```

检查包与可执行文件：

```bash
ros2 pkg prefix lvi_sam
ros2 pkg executables lvi_sam
```

第一阶段应看到 `lvi_sam_imuPreintegration` 与 `lvi_sam_mapOptimization`。相机标定完成后，
再运行 `bash scripts/build.sh --clean` 构建完整 VIS；Orin 的 OpenCV ABI 核验见
[`DEPLOY_ORIN.md`](DEPLOY_ORIN.md#3-opencv-冲突决策orin-头号坑)。

应至少看到：

- `lvi_sam_imuPreintegration`
- `lvi_sam_mapOptimization`
- `visual_feature_node`
- `visual_estimator_node`
- `visual_loop_node`

如果是 AGX Orin，先按 [DEPLOY_ORIN.md](DEPLOY_ORIN.md) 检查 JetPack/L4T、swap 和 OpenCV ABI，再运行 `bash scripts/setup_orin.sh`。

## 3. 启动前必须确认的配置

不要直接把示例标定值用于正式运行。至少核对以下内容：

1. `pointCloudTopic` 实际发布 `livox_ros_driver2/msg/CustomMsg`。
2. `imu_topic` 实际发布 `sensor_msgs/msg/Imu`，时间戳与雷达使用同一时钟基准。
3. `lidarFrame`、`baselinkFrame`、`odometryFrame`、`mapFrame` 与机器人 TF 树一致。
4. `extrinsicRot`、`extrinsicRPY`、`extrinsicTrans` 是当前设备的雷达—IMU 标定结果。
5. 启用视觉时，`params_camera.yaml` 中图像尺寸、模型、内参、畸变和相机—IMU/雷达外参均为当前相机标定结果。
6. 地图目录存在且运行用户可写；定位时目录中是完整、同一批次生成的地图文件。
7. RTK 输入已经变换到保存地图所使用的局部坐标系；仅把 `frame_id` 改成 `odom` 并不等于完成了坐标对齐。

推荐先检查话题和 TF：

```bash
ros2 topic info /livox/lidar -v
ros2 topic info /IMU_data -v
ros2 topic hz /livox/lidar
ros2 topic hz /IMU_data
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo base_link imu_link
```

## 4. 分阶段测试流程

### 4.1 第一阶段：纯激光—IMU 冒烟测试

先启动 MID360 驱动，再关闭视觉运行通用建图配置：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

另开终端：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch lvi_sam run.launch.py \
  mode:=mapping \
  scene:=generic \
  enable_visual:=false \
  pcd_directory:=/home/$USER/maps/lvi_test \
  robot_description_file:=/absolute/path/to/robot.urdf.xacro \
  use_sim_time:=false
```

运行期间检查：

```bash
ros2 topic hz /lio_sam/mapping/odometry
ros2 topic hz /lio_sam/mapping/cloud_registered
ros2 topic echo /lio_sam/localization/state --once
```

通过标准：节点不退出，里程计连续，静止时无明显持续漂移，运动方向与 RViz/实车一致，TF 中不存在两个节点同时发布同一条 `odom -> base_link`。

### 4.2 第二阶段：建图与回环

普通场景使用 `scene:=generic`；充电/铁路场景、且上游 RTK 对齐节点已经可用时使用 `scene:=charging`。

```bash
ros2 launch lvi_sam run.launch.py \
  mode:=mapping \
  scene:=generic \
  enable_visual:=false \
  pcd_directory:=/home/$USER/maps/site_001 \
  robot_description_file:=/absolute/path/to/robot.urdf.xacro
```

采集时让机器人经过可识别结构并回到已走过区域。正常退出节点后检查地图：

```bash
MAP_DIR=/home/$USER/maps/site_001
test -s "$MAP_DIR/trajectory.pcd"
test -s "$MAP_DIR/transformations.pcd"
test -s "$MAP_DIR/map_manifest.yaml"
find "$MAP_DIR/Scans" -name '*.pcd' | wc -l
find "$MAP_DIR/SCDs" -name '*.scd' | wc -l
find "$MAP_DIR/CornerMap" -name '*.pcd' | wc -l
find "$MAP_DIR/SurfMap" -name '*.pcd' | wc -l
```

四类关键帧文件的编号应连续，并与 `map_manifest.yaml` 中的 `keyframe_count` 一致。不要在节点仍在保存时关机或移动地图目录。

### 4.3 第三阶段：加载先验地图定位

```bash
ros2 launch lvi_sam run.launch.py \
  mode:=localization \
  scene:=generic \
  enable_visual:=false \
  pcd_directory:=/home/$USER/maps/site_001 \
  robot_description_file:=/absolute/path/to/robot.urdf.xacro
```

验证状态和强制重定位：

```bash
ros2 topic echo /lio_sam/localization/state
ros2 service call /lio_sam/localization/force_relocalize std_srvs/srv/Trigger '{}'
```

首次定位应从 `Initializing` 进入 `Initialized`。遮挡雷达或从较差初值启动时，应能报告丢失并重新执行 Scan Context + ICP，而不是继续发布看似正常但已经错误的位姿。

### 4.4 第四阶段：视觉链路

完成纯激光测试后再启用相机：

```bash
ros2 launch lvi_sam run.launch.py \
  mode:=mapping \
  scene:=generic \
  enable_visual:=true \
  camera_params_file:=$(ros2 pkg prefix lvi_sam)/share/lvi_sam/config/params_camera.yaml \
  pcd_directory:=/home/$USER/maps/visual_test \
  robot_description_file:=/absolute/path/to/robot.urdf.xacro
```

检查数据链：

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /odometry/imu
ros2 topic hz /lio_sam/deskew/cloud_deskewed
ros2 topic echo /lvi_sam/vins/loop/match_frame --once
```

`match_frame` 只在 DBoW2/BRIEF 找到并验证回环时发布，因此短时间没有消息不代表节点异常。应同时观察三个视觉节点日志、特征跟踪图和 VINS 里程计是否连续。

当前版本会保存相机原始数据的设计约定和地图清单入口，但没有实现跨进程持久化 DBoW2/BRIEF 数据库；跨会话全局重定位仍由 Scan Context + ICP 主导。不要把“在线视觉回环可用”误认为“重启后可用视觉地图重定位”。

### 4.5 第五阶段：RTK 建图和定位

本包订阅的是地图对齐后的 `nav_msgs/msg/Odometry`，不会在内部把原始经纬度自动对齐到历史地图。上游转换节点需要负责：

- 经纬度到局部 ENU/UTM 的转换；
- 建图原点、旋转和平移的持久化；
- 天线杆臂补偿；
- 接收机 FIX/FLOAT/无解状态转换；
- 位置与航向协方差；
- 输出时间戳及 `frame_id`。

充电场景当前约定话题 `/gps/lio_sam_odom`、坐标帧 `odom`：

```bash
ros2 topic info /gps/lio_sam_odom -v
ros2 topic hz /gps/lio_sam_odom
ros2 topic echo /gps/lio_sam_odom --once
```

重点核对：

- `header.frame_id == odom`；
- `pose.covariance[0]`、`[7]` 为正且小于 `gpsCovThreshold`；
- 使用航向时四元数已归一化，`pose.covariance[35]` 为正且小于 `Loc.rtkYawVarianceThreshold`；
- RTK 与激光时间差小于 `gpsTimeTolerance`；
- RTK 坐标在 RViz 中与先验地图和激光定位重合。

建图：

```bash
ros2 launch lvi_sam run.launch.py \
  mode:=mapping scene:=charging enable_visual:=false \
  gps_topic:=/gps/lio_sam_odom \
  pcd_directory:=/home/$USER/maps/charging_001 \
  robot_description_file:=/absolute/path/to/robot.urdf.xacro
```

定位：

```bash
ros2 launch lvi_sam run.launch.py \
  mode:=localization scene:=charging enable_visual:=false \
  gps_topic:=/gps/lio_sam_odom \
  pcd_directory:=/home/$USER/maps/charging_001 \
  robot_description_file:=/absolute/path/to/robot.urdf.xacro
```

日志出现 `Rejecting GPS factor input` 或 `Rejecting RTK localization sample/assist` 时，说明质量门控正在拒绝异常数据；应根据后面的 frame、covariance、time 或 innovation 原因修复上游数据，不能简单放宽全部阈值。

## 5. 建议记录的测试结果

每次测试保留以下信息：

```bash
git rev-parse HEAD
uname -a
printenv ROS_DISTRO
ros2 doctor --report
ros2 bag record \
  /livox/lidar /IMU_data /camera/image_raw /gps/lio_sam_odom \
  /lio_sam/mapping/odometry /lio_sam/localization/state
```

建议至少记录：启动成功率、首次重定位耗时、重复路线闭环误差、定位丢失次数、RTK 接收/拒绝数量、CPU/内存峰值，以及断开相机或 RTK 后激光主链是否继续稳定工作。

## 6. 已知边界

- 本次桌面端完成了 Python/YAML/XML/CMake 源码结构、配置一致性、脚本语法、Git 差异和关键 C++ 控制流检查；桌面没有正在运行的 Docker Linux 引擎，也没有 ROS 2 Humble，因此最终编译和传感器运行结果必须以远程 Ubuntu/Orin 为准。
- 示例相机和雷达外参不能替代实机标定。
- 视觉地图数据库尚未跨会话持久化。
- RTK `frame_id` 一致只是输入契约的一部分，真正有效的 RTK 融合必须保证坐标数值与先验地图对齐。
- 高度重复的铁路、站台或长直走廊仍可能产生感知混淆；Scan Context/视觉候选都必须结合 ICP、RTK 和连续轨迹一致性评估。

更详细的模块职责与地图格式见 [ARCHITECTURE_AND_MAP_FORMAT.md](ARCHITECTURE_AND_MAP_FORMAT.md)，参数解释和常见故障见 [USAGE.md](USAGE.md)。
