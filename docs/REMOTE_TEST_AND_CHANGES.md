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
- 使用工程内图像适配层替代 VIS 对 `cv_bridge` 的直接链接，支持当前实机 `rgb8`
  及常见 8 位 ROS 图像编码，避免 Orin 同一进程混载 OpenCV 4.5/4.8。

### 1.3 ROS 2 与 MID360

- 工程包、CMake、安装规则和依赖统一为 ROS 2 Humble 的 `lvi_sam` 包。
- launch 支持参数文件自动选择、两套 IMU profile、统一 RTK 话题、可关闭视觉节点、可选 URDF/Xacro 和仿真时间。
- MID360 使用 `livox_ros_driver2/msg/CustomMsg`，并保留原项目已经验证过的 MID360 前端逻辑。
- 加入传感器类型、扫描规模、量程、体素、IMU 噪声、外参矩阵和 TF 新鲜度的启动期校验，错误配置会尽早失败并给出原因。

### 1.4 RTK 融合

- 建图 `useGpsFactor` 与定位 `Loc.useRTKAssist` 分开控制，默认不隐式启用全局因子。
- 输入统一为已经与 LIO 地图对齐的 `nav_msgs/Odometry`，并检查时间、`frame_id`、有限值和协方差。
- 建图 GPS 因子增加距离/创新量门限、方差下限和 Huber robust kernel。
- 定位初始化要求多帧稳定 RTK；在线辅助使用绝对创新量、归一化创新量及自适应融合权重。
- 可选双天线航向，但只有四元数和 yaw 协方差都可信时才接收。

### 1.5 工程稳健性与接口规范

- 视觉优化内部的 `Estimator`、`FeatureManager`、`InitialEXRotation` 和 `IMUFactor`
  改为普通 C++ 对象，避免每个优化因子创建 ROS Node。
- 修复视觉估计工作线程退出时未 join、恢复 join 后条件变量无法唤醒的问题；节点现在可确定退出。
- BRIEF pattern 由视觉回环模块直接读取并强制校验 256 对测试点；词表和 pattern 缺失时启动失败。
- VIS 输入增加有限值、时间顺序、特征通道长度和队列上限检查，固定资源改用 RAII。
- `project_name`、`odom_topic` 成为统一 launch 接口；VIS 话题由
  `include/lvi_sam/topic_names.hpp` 集中生成。
- 资源包名与视觉话题根已解耦，`project_name:=robot_a` 不再导致节点错误查找 `robot_a` 包。
- 视觉关键帧三流同步和 VIS→LIS 候选映射增加显式时间容差；超限候选不会进入 ICP。
- 安装脚本会复用 `/usr/local` 或系统路径下已有的 GTSAM 4.x，不再向系统 Python 安装
  未使用的 `opencv-python`/`numpy`。
- 新增 `.gitattributes` 固定 ROS/CMake/Python/Shell 文本为 LF，避免从 Windows 提交后在 Orin
  出现脚本解释器 `^M` 或无意义的整文件换行差异。
- 新增 `scripts/validate_config.py`，`scripts/build.sh` 会在编译前自动核对六套 LIS 配置、两套 IMU profile 与
  camera/BRIEF 配置；`--lidar-only` 构建会只核对 LIS。完整契约见
  [INTERFACES_AND_STABILITY.md](INTERFACES_AND_STABILITY.md)。
- 修复 LIS→VIS 初始化元数据链：图优化重置编号和退化标志不再共用同一语义，IMU 预积分恢复发布
  加速度计/陀螺仪 bias 与重力；VIS 对缺失、非有限或零重力先验执行拒绝，不再把无效值送入初始化。
  兼容字段的逐槽位说明见接口文档 3.2.1。
- 去畸变、特征提取和地图优化现在接收同一份 ROS NodeOptions/YAML 覆盖值，避免内部前端静默回退到
  默认扫描模型、外参或体素参数，而地图后端使用实机配置。
- 构建脚本会保留调用者已经 source 的工作区环境，从而优先复用机器人总工作区中的
  `livox_ros_driver2`；仓库位于 `<workspace>/src` 时自动使用上层工作区，`--clean` 只清理
  `build/lvi_sam` 与 `install/lvi_sam`。
- 地图落盘改为 executor、回环线程和全局可视化线程全部退出后再执行，避免关机保存与回环修正并发读取
  关键帧/因子图容器；逐关键帧 PCD/SCD 或最终关键 PCD 任一写入失败时不会更新 manifest，也不会误报完成。输出目录必须为空，
  建图期间的 `.lvi_sam_mapping_in_progress` 标记会阻止定位加载未完成地图。应使用 Ctrl+C
  正常退出并等待“Saving map ... completed”。
- VIS Estimator 的预积分、边缘化和 `ImageFrame` 拥有型指针在构造时全部显式置空，并在析构时统一清理，
  消除 Release 优化下由未初始化指针判断导致的随机崩溃风险。
- BRIEF 词表读取增加头字段、数量上限、文件剩余长度和完整读取检查；损坏或截断的 60 MB 词表会在
  启动阶段明确失败，不再按伪造数量分配内存或把不完整描述子送入 DBoW2。
- 视觉初始化的 IMU 可观测性统计现在从零向量开始，只使用有限且正时长的预积分区间；没有有效区间时
  明确拒绝初始化，避免未初始化内存或除零把随机姿态带入后端。
- 回环候选历史点云与 ICP 校正后点云拆分为两个诊断话题，避免不同坐标状态的数据在同一话题交替发布。

### 1.6 定位状态与跨节点复位

- 新增 `lvi_sam_msgs/msg/LocalizationStatus`，通过 `/lio_sam/localization/status` 输出
  `MAPPING`、`RELOCALIZING`、`VERIFYING`、`TRACKING`、`DEGRADED`、`LOST` 六类结构化状态；
  旧的 `/lio_sam/localization/state` 字符串话题继续保留。
- 新增 `lvi_sam_msgs/msg/LocalizationReset`，通过 `/lio_sam/localization/reset` 传递
  地图重定位、强制重定位、回环图修正、VINS/IMU 失败等事件。`mapOptimization` 是唯一的
  `reset_id` 所有者，接收节点按代次清理旧队列，避免旧 DDS 样本跨复位进入新估计。
- 状态接口是上层导航的观察/门控接口；复位接口是节点协同事件。两者都不是 Nav2 速度命令，
  也不替代 `map -> odom -> base_link` TF。
- 验收必须分两阶段执行：先验证状态输出和旧 LIS 结果，再验证 IMU/VIS 队列清理、时间戳水位线、
  VINS 重启和低代次事件丢弃。详见 `LOCALIZATION_ACCEPTANCE_MATRIX.md`。

### 1.7 明确的兼容性边界

- 合法输入下，点云匹配、IMU 预积分、因子图、VINS 滑窗和原有回环求解器未被替换；新增代码
  主要负责配置、输入校验、线程生命周期、地图事务和接口输出。
- 视觉默认启动是 launch 策略，不代表 LIS 依赖相机；纯激光排障仍使用
  `enable_visual:=false`，纯 LIS 构建仍使用 `-DBUILD_VISUAL=OFF`。
- 所有物理外参都来自 YAML profile。更换 IMU、雷达安装位置或相机时优先复制/修改对应 profile，
  不在 C++ 中添加设备专用常量，也不依赖 TF 查询作为外参兜底。
- Windows 审阅环境只能完成配置、脚本和静态检查；没有 Orin 的真实运行日志时，不能声称
  `LOST`、强制重定位、VINS 失败复位和低质量 RTK 已通过实机验收。

## 2. 远程电脑首次部署

### 2.1 拉取代码

```bash
cd ~/work
git clone https://github.com/foreverXyoung/LVI-SAM-ROS2-Enhanced.git
cd LVI-SAM-ROS2-Enhanced

# 已经克隆过时使用：
git pull --ff-only origin main
# 仅在当前已 source 的工作区找不到 livox_ros_driver2 时执行：
# git submodule update --init --recursive
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

如果仓库位于现有机器人工作区 `/data/return_station_ws/src/`，应从工作区根目录重编，
避免同时维护仓库内外两套 `build/install`：

```bash
cd /data/return_station_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --symlink-install \
  --packages-select lvi_sam \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_VISUAL=ON

source install/setup.bash
colcon test --packages-select lvi_sam --event-handlers console_direct+
colcon test-result --verbose
```

也可以直接在仓库目录执行 `bash scripts/build.sh --clean`；脚本会识别上层
`/data/return_station_ws`，自动复用其 `install/livox_ros_driver2` 并只清理
`lvi_sam` 自身产物。

本次内部图像适配层不要求重新编译系统 `cv_bridge`，也不需要额外 overlay 工作空间。

检查包与可执行文件：

```bash
ros2 pkg prefix lvi_sam
ros2 pkg executables lvi_sam
```

第一阶段应看到 `lvi_sam_imuPreintegration` 与 `lvi_sam_mapOptimization`。相机标定完成后，
再运行 `bash scripts/build.sh --clean` 构建完整 VIS；Orin 的 OpenCV 单一依赖核验见
[`DEPLOY_ORIN.md`](DEPLOY_ORIN.md#3-opencv-单一依赖策略)。

应至少看到：

- `lvi_sam_imuPreintegration`
- `lvi_sam_mapOptimization`
- `visual_feature_node`
- `visual_estimator_node`
- `visual_loop_node`

如果是 AGX Orin，先按 [DEPLOY_ORIN.md](DEPLOY_ORIN.md) 检查 JetPack/L4T、swap 和 OpenCV，再运行 `bash scripts/setup_orin.sh`。

## 3. 启动前必须确认的配置

不要直接把示例标定值用于正式运行。至少核对以下内容：

1. `pointCloudTopic` 实际发布 `livox_ros_driver2/msg/CustomMsg`。
2. `imu_source` 选择的 profile 与物理 IMU 一致，实际话题发布 `sensor_msgs/msg/Imu`，
   时间戳与雷达使用同一时钟基准；MID-360 原始加速度约为 `1 g` 是正常现象。
3. `lidarFrame`、`baselinkFrame`、`odometryFrame`、`mapFrame` 与机器人 TF 树一致。
4. `extrinsicRot`、`extrinsicRPY`、`extrinsicTrans` 是当前设备的雷达—IMU 标定结果。
5. 启用视觉时，所选 `params_camera*.yaml` 中图像尺寸、模型、内参、畸变和相机—IMU/雷达外参均与当前设备对应；MID-360 文件已写入实机 `T_cam_radar`，但内置 IMU 杆臂仍需验收。
6. 建图输出目录为空且运行用户可写；定位时目录中是完整、同一批次生成且没有
   `.lvi_sam_mapping_in_progress` 标记的地图文件。
7. RTK 输入已经变换到保存地图所使用的局部坐标系；仅把 `frame_id` 改成 `odom` 并不等于完成了坐标对齐。

推荐先检查话题和 TF：

```bash
ros2 topic info /livox/lidar -v
ros2 topic info /IMU_data -v
ros2 topic info /livox/imu -v
ros2 topic hz /livox/lidar
ros2 topic hz /IMU_data
ros2 topic hz /livox/imu
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
  imu_source:=external \
  enable_visual:=false \
  publish_fused_tf:=false \
  pcd_directory:=/home/$USER/maps/lvi_test \
  use_sim_time:=false
```

保持机器人静止并记录外置 IMU 结果后，使用新的空地图目录切换到 MID-360 内置 IMU：

```bash
ros2 launch lvi_sam run.launch.py \
  mode:=mapping \
  scene:=generic \
  imu_source:=mid360 \
  enable_visual:=false \
  publish_fused_tf:=false \
  pcd_directory:=/home/$USER/maps/lvi_mid360_imu_test \
  use_sim_time:=false
```

两次测试不能复用地图目录。先比较静止漂移、重力模长、姿态和短距离闭环误差，再决定正式
使用哪一套 IMU；MID-360 profile 的旋转和平移参考 FAST-LIO 官方 Mid-360 配置，噪声参数仍是待标定初值。

运行期间检查：

- 若日志出现 `Reordered Livox frame with non-monotonic point offsets`，说明驱动组帧中存在
  包级时间乱序；转换层已排序并继续处理。该警告持续高频出现时仍应检查网口丢包、时间同步
  和 Livox ROS Driver 2 版本。

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
  imu_source:=mid360 \
  enable_visual:=true \
  enable_rviz:=true \
  image_topic:=/camera/color/image_raw \
  pcd_directory:=/home/$USER/maps/visual_test \
  use_sim_time:=false \
  publish_fused_tf:=false
```

检查数据链：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /odometry/imu
ros2 topic hz /lio_sam/deskew/cloud_deskewed
ros2 topic echo /lvi_sam/vins/loop/match_frame --once
```

`match_frame` 只在 DBoW2/BRIEF 找到并验证回环时发布，因此短时间没有消息不代表节点异常。应同时观察三个视觉节点日志、特征跟踪图和 VINS 里程计是否连续。

仓库当前相机配置按 `640x400`、PINHOLE/plumb_bob 编写；必须用实机
`/camera/color/camera_info` 复核，不能仅凭图像编码判断内参正确：

```bash
ros2 topic echo /camera/color/camera_info --once
```

`/camera/color/image_raw` 已确认编码为 `rgb8`。内部适配层按 `RGB → mono8` 一次转换，
生成的灰度矩阵持有独立内存，可安全用于跨帧光流。
`imu_source:=mid360` 会自动选择 `params_camera_mid360.yaml`。当前文件已把实机
`T_cam_radar` 转换到算法虚拟深度帧并启用 LiDAR 深度；相机—IMU 外参继续在线细化，
LIS 里程计先验和全局视觉—激光对齐暂不启用。先验证深度投影与 VIO，再逐项增加耦合链路。

当前版本仅定义了相机重定位数据的设计约定，并在地图清单中预留入口；代码不会实际保存相机原图、描述子或跨进程 DBoW2/BRIEF 数据库。跨会话全局重定位仍由 Scan Context + ICP 主导。不要把“在线视觉回环可用”误认为“重启后可用视觉地图重定位”。

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
- RTK 时间戳严格递增，不允许重复、回拨或非有限值；
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

日志出现 `Rejecting GPS factor input`、`Rejecting RTK localization sample/assist` 或
`Discarding RTK/GPS sample` 时，说明质量门控正在拒绝异常数据；应根据后面的
frame、covariance、timestamp、time 或 innovation 原因修复上游数据，不能简单放宽全部阈值。

## 5. 建议记录的测试结果

每次测试保留以下信息：

```bash
git rev-parse HEAD
uname -a
printenv ROS_DISTRO
ros2 doctor --report
ros2 bag record \
  /livox/lidar /IMU_data /camera/color/image_raw /gps/lio_sam_odom \
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
