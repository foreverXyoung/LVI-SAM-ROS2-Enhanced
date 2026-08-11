# 接口、依赖与稳定性契约

本文档是 LVI-SAM-ROS2-Enhanced 的工程契约。后续升级算法、替换驱动或增加重定位方法时，
应优先保持本文中的话题语义、时间/坐标约束和降级行为；确需破坏兼容性时，应同步修改
配置预检器、launch、README 和本文档。

## 1. 模块边界

| 模块 | 可执行程序 | 职责 | 不负责的内容 |
|---|---|---|---|
| LIS IMU | `lvi_sam_imuPreintegration` | IMU 预积分、发布高频里程计 | 相机处理、地图匹配 |
| LIS Map | `lvi_sam_mapOptimization` | Mid-360 前端、因子图、建图、先验地图定位、Scan Context、RTK/GNSS 因子 | 相机特征提取 |
| VIS Feature | `visual_feature_node` | 图像灰度适配、特征跟踪、可选激光深度注册 | 位姿优化、回环判定 |
| VIS Estimator | `visual_estimator_node` | VIO 滑窗优化、接收 LIS 尺度/位姿先验 | 先验地图装载 |
| VIS Loop | `visual_loop_node` | BRIEF + DBoW2 回环候选生成 | 直接修改 LIS 因子图 |

VIS 通过话题与 LIS 松耦合。视觉计算类 `Estimator`、`FeatureManager`、
`InitialEXRotation`、`IMUFactor` 是普通 C++ 对象，不得继承或动态创建 ROS Node。
VIS 侧 ROS 图中只允许出现上述进程入口创建的节点。

LIS 当前为兼容合并式前端，`ImageProjection` 与 `FeatureExtraction` 由 Map 进程内部持有；
它们必须接收和 `mapOptimization` 完全相同的 `NodeOptions`，保证 YAML 覆盖值在扫描投影、
特征提取和地图后端一致。两个兼容参数上下文使用唯一节点名
`lvi_sam_image_projection_internal`、`lvi_sam_feature_extraction_internal`，不得被进程级
`__node` remap 合并成重名节点。后续将其改为纯计算类时，应由显式配置结构体注入，不得重新读取默认参数。

## 2. 启动入口契约

统一入口：

```bash
ros2 launch lvi_sam run.launch.py \
  mode:=mapping \
  project_name:=lvi_sam \
  imu_topic:=/IMU_data \
  odom_topic:=/odometry/imu \
  image_topic:=/camera/color/image_raw
```

关键参数：

| 参数 | 默认值 | 契约 |
|---|---:|---|
| `mode` | `mapping` | 仅允许 `mapping` / `localization` |
| `scene` | `generic` | 仅允许 `generic` / `charging` / `gazebo` |
| `project_name` | `lvi_sam` | 仅作为 VIS 绝对话题根，生成 `/<name>/vins/...`；不作为 ROS 包名 |
| `imu_topic` | `/IMU_data` | 同时覆盖 LIS 与 VIS，禁止为空 |
| `odom_topic` | `/odometry/imu` | LIS 预积分输出与 VIS 先验输入共用，禁止为空 |
| `image_topic` | `/camera/color/image_raw` | `enable_visual:=true` 时必须有效 |
| `gps_topic` | 空 | 非空时覆盖 LIS 的 `gpsTopic` |
| `enable_visual` | `true` | 可关闭整套 VIS；不影响纯激光运行 |
| `enable_rviz` | `true` | 可独立关闭，适用于无桌面的 SSH 环境 |
| `publish_fused_tf` | `true` | 控制算法是否发布 `odom -> base_link` |
| `publish_map_odom_static` | `false` | 仅在系统中没有其他 `map -> odom` 发布者时开启 |

所有布尔 launch 参数只接受 `true` / `false`。地图目录、LIS/VIS 配置、RViz 配置和可选
URDF/Xacro 都必须使用绝对路径。入口会在创建节点前检查文件以及必填参数，错误时立即失败，
不依赖调用终端的当前目录，也不以默认空值继续运行。
`camera_params_file` 可使用真实绝对路径；launch 会把同一文件传给 Feature、Estimator 和 Loop。
词表、BRIEF pattern、鱼眼 mask 支持包内相对路径或真实存在的绝对路径，旧版 `/config/...`
包内写法仍兼容。

## 3. 话题接口

### 3.1 外部输入

| 话题（默认） | 类型 | 消费者 | QoS/语义 |
|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | LIS Map | 传感器流；点时间偏移必须属于本帧时间基准 |
| `/IMU_data` | `sensor_msgs/msg/Imu` | LIS IMU、LIS Map、VIS Estimator | 传感器 QoS；时间戳严格递增；单位为 SI |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | VIS Feature、VIS Loop | 传感器 QoS；支持 `rgb8/bgr8/rgba8/bgra8/mono8/8UC1` |
| `gpsTopic` | `nav_msgs/msg/Odometry` | LIS Map | 可选；上游负责 RTK 质量、坐标投影与杆臂补偿 |
| `externalPoseTopic` | `nav_msgs/msg/Odometry` | LIS Map | 可选外部位姿；按配置门限验收 |

图像适配由 `include/lvi_sam/image_conversion.hpp` 完成。LVI-SAM 可执行程序不链接
`cv_bridge`，从而避免同一进程同时装载 ROS OpenCV 4.5 与 JetPack OpenCV 4.8。

### 3.2 LIS 输出与内部接口

| 话题 | 类型 | 说明 |
|---|---|---|
| `/odometry/imu` | `nav_msgs/msg/Odometry` | IMU 预积分融合输出；pose 的 child frame 为 `lidarFrame`，同时是 VIS 的尺度/位姿先验 |
| `/odometry/imu_incremental` | `nav_msgs/msg/Odometry` | 高频增量里程计；pose 已由 IMU 转到 `lidarFrame`，供 LIS Map 使用 |
| `/lio_sam/mapping/odometry` | `nav_msgs/msg/Odometry` | 优化后的物理 LiDAR 位姿，`child_frame_id=lidarFrame` |
| `/lio_sam/mapping/odometry_incremental` | `nav_msgs/msg/Odometry` | 地图优化增量结果，`child_frame_id=lidarFrame` |
| `/lio_sam/deskew/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | 去畸变点云，可选供 VIS 深度注册 |
| `/lio_sam/mapping/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 已注册点云，供 RViz/下游定位使用 |
| `/lio_sam/mapping/icp_loop_closure_history_cloud` | `sensor_msgs/msg/PointCloud2` | 回环候选的历史局部点云，仅用于诊断 |
| `/lio_sam/mapping/icp_loop_closure_corrected_cloud` | `sensor_msgs/msg/PointCloud2` | ICP 对齐后的回环点云，仅用于诊断 |
| `/lio_sam/localization/state` | `std_msgs/msg/String` | 定位状态机输出 |
| `/lio_sam/localization/force_relocalize` | `std_srvs/srv/Trigger` | 强制重新进入全局重定位流程 |

#### 3.2.1 LIS→VIS 内部里程计元数据

为了兼容原版 LVI-SAM 的紧耦合初始化，两个**内部** `nav_msgs/msg/Odometry`
接口暂时在 `pose.covariance` 的前若干槽位携带控制/状态元数据。这些槽位不是统计协方差，
其索引统一定义在 `include/lvi_sam/internal_odom_metadata.hpp`，禁止在其他源码中继续写魔法数字。

| 话题 | `pose.covariance` 契约 |
|---|---|
| `/lio_sam/mapping/odometry_incremental` | `[0]` 图优化重置编号；`[1]` 退化标志（0/1） |
| `/odometry/imu_incremental`、`/odometry/imu` | `[0]` 图优化重置编号；`[1..3]` 加速度计 bias；`[4..6]` 陀螺仪 bias；`[7]` 重力模长 |

图优化在 GPS、外部位姿或回环导致历史位姿重算后递增重置编号；IMU 预积分检测到编号变化时
先清空旧积分状态，再从新图优化序列初始化。VIS 只接收时间接近图像、所有字段有限、
重置编号为非负整数且重力位于 5–15 m/s² 的先验，缺失元数据时拒绝该先验，不能以零重力继续优化。

`/odometry/imu` 因兼容原因同时承担融合里程计和 VIS 内部先验。通用导航消费者如需统计协方差，
不应解释上述槽位，应使用 `/lio_sam/mapping/odometry` 或在系统集成层发布带真实协方差的独立里程计。
后续若引入自定义接口包，应以具名消息字段替换该兼容编码，并保留一版桥接期。

### 3.3 VIS 话题

以下 `<root>` 为 `/<project_name>`，默认 `/lvi_sam`。话题名由
`include/lvi_sam/topic_names.hpp` 统一生成，模块内不得再手写另一套前缀。

| 话题 | 类型 | 生产者 → 消费者 |
|---|---|---|
| `<root>/vins/feature/feature` | `sensor_msgs/msg/PointCloud` | Feature → Estimator |
| `<root>/vins/feature/restart` | `std_msgs/msg/Bool` | Feature → Estimator |
| `<root>/vins/depth/depth_feature` | `sensor_msgs/msg/PointCloud2` | Feature 调试输出 |
| `<root>/vins/odometry/odometry` | `nav_msgs/msg/Odometry` | Estimator → 外部 |
| `<root>/vins/odometry/keyframe_pose` | `nav_msgs/msg/Odometry` | Estimator → Loop |
| `<root>/vins/odometry/keyframe_point` | `sensor_msgs/msg/PointCloud` | Estimator → Loop |
| `<root>/vins/odometry/extrinsic` | `nav_msgs/msg/Odometry` | Estimator → Loop |
| `<root>/vins/loop/match_frame` | `std_msgs/msg/Float64MultiArray` | Loop → LIS Map |

`match_frame.data[0]` 是当前关键帧时间戳，`data[1]` 是匹配历史关键帧时间戳；消息当前严格包含
这两个字段。它是“候选约束”接口，LIS 仍应执行几何一致性检查，
不能把词袋相似度直接当作可信闭环。

视觉回环的三路关键帧输入由 `loop_sync_tolerance`（默认 0.02 s）约束；候选时间戳映射到
LIS 关键帧时由 `externalLoopTimeTolerance`（默认 0.2 s）约束。任一时间差超限即丢弃，
通过时间门限的候选还必须通过 LIS 的点云数量、ICP 收敛和适应度门限。
DBoW2 的近期关键帧排除量和候选分数分别由 `loop_min_index_gap`、
`loop_primary_score_threshold`、`loop_secondary_score_threshold` 配置并在启动前校验。

## 4. 时间与坐标契约

- LiDAR、IMU、camera、RTK 必须使用同一时间基准。若无法硬同步，只有在实测确认后才开启
  `estimate_td`；它不能修复跳变或来自不同系统时钟的数据。
- `/IMU_data` 的角速度单位为 rad/s、线加速度为 m/s²。非有限值和倒序时间戳会被 VIS
  丢弃，不能进入优化器。
- `lidarFrame`、`baselinkFrame`、`odometryFrame`、`mapFrame` 是 LIS 内部及输出契约。
  `publish_fused_tf:=false` 时算法不依赖平台 TF 树完成核心积分，但外参仍必须通过 YAML
  提供正确标定值。
- VIS 当前内部固定使用 `vins_world`、`vins_body`、`vins_body_ros` 三个兼容帧；它们不替代
  LIS/Nav2 的 `map`、`odom`、`base_link`。其中 `vins_body_ros` 是位于 VINS camera/body
  原点、但采用 ROS/LiDAR 轴向的**虚拟深度投影帧**，不是物理雷达安装坐标系。
  在统一 TF 架构前，不应让 VIS 单独承担 Nav2 TF。
- `extrinsicRot` / `extrinsicRPY` / `extrinsicTrans` 属于 IMU-LiDAR 标定；
  `extrinsicRotation` / `extrinsicTranslation` 属于 camera-IMU 标定；
  `lidar_to_cam_*` 属于 LiDAR-camera 标定。为兼容原版保留了该参数名，其严格语义是
  “物理 LiDAR → `vins_body_ros` 虚拟深度帧”的完整 SE(3)，只对原始点云应用一次；
  camera/VINS 与 ROS 轴向之间的固定旋转由公共坐标约定模块统一处理。三组参数不能互相替代。
- VIS 使用的 LIS 里程计输入必须表达物理 LiDAR 在 ROS `odom` 中的位姿，线速度必须在
  ROS `odom` 世界坐标系表达。初始化会先应用 LiDAR-camera 外参，再应用固定的
  ROS-odom → VINS-world 坐标约定；速度只做世界坐标转换，不错误叠加传感器外参旋转。
- 未完成 LiDAR-camera 标定前保持 `use_lidar: 0`、
  `use_lidar_odometry_prior: 0` 和
  `align_camera_lidar_estimation: 0`，否则错误深度会系统性污染视觉特征。

## 5. 构建与依赖契约

| 层 | 依赖 | 策略 |
|---|---|---|
| ROS 基础 | Humble、rclcpp/rclpy、消息、tf2、PCL ROS 适配 | 由 rosdep/apt 安装 |
| 数学/点云 | Eigen3、PCL、OpenMP | CMake 目标级链接；OpenMP 仅用于地图优化 |
| 因子图 | GTSAM `>=4.0,<5` | 可使用现有 4.1.x；安装脚本默认构建 4.0.3 |
| 视觉 | OpenCV、Ceres、Boost | 仅 `BUILD_VISUAL=ON` 时构建 Ceres/VIS 目标 |
| 驱动消息 | `livox_ros_driver2` | 优先复用已 source 的机器人工作区；否则使用子模块 |
| Python 运行时 | rclpy、PyYAML、pyproj、ament_index_python | 仅脚本/launch 运行依赖，不参与 C++ CMake 查找 |

`package.xml` 使用标准 rosdep 键 `libopencv-dev`、`libpcl-all-dev`、`libceres-dev`
和 `boost`。CMake 不查找 `rclpy`；Python 依赖以 `exec_depend` 声明。

构建前自动运行：

```bash
python3 src/lvi_sam/scripts/validate_config.py \
  --config-dir src/lvi_sam/config

# 纯 LIS 构建只检查激光侧契约
python3 src/lvi_sam/scripts/validate_config.py \
  --config-dir src/lvi_sam/config --lidar-only
```

该工具核对六套 LIS 配置的传感器话题、帧名、外参数组与 mapping/localization 标志，
并检查 VIS 数值范围、LIS/VIS IMU/里程计接口一致性、词表和 256 对 BRIEF pattern。

## 6. 失败隔离与降级

- `BUILD_VISUAL=OFF`：编译和运行纯 LIS，不要求 Ceres/Boost 参与视觉目标链接。
- `enable_visual:=false`：运行时关闭 VIS，LIS 建图/定位不受影响。
- `use_lidar: 0`（camera YAML）：只关闭 VIS 的激光深度投影，不关闭 LIS。
- `use_lidar_odometry_prior: 0`：只关闭 LIS→VIS 的里程计初始化先验；该开关与
  激光深度投影独立，但两者都依赖正确的 LiDAR-camera 标定。
- `loop_closure: 0`：视觉回环节点不创建工作线程并正常退出。
- 图像编码、尺寸、步长或数据长度错误：丢弃该帧并节流告警。
- VIS 特征消息通道不足、非有限值或 IMU 时间序列非法：丢弃该帧，不把坏数据送入优化器。
- VIS 初始化只统计有限且正时长的 IMU 预积分区间；没有有效区间时保持未初始化并报告原因。
- VIS 重启会原子清空 IMU、特征、LIS 先验队列并复位预测状态；相机断流/时间回拨还会清空
  特征跟踪历史、深度缓存和当前视觉词袋序列，避免跨坐标序列匹配；退出时工作线程必须 join。
- RTK/GPS 输入必须使用有限且严格递增的时间戳；位置、航向、协方差、时间差和创新量逐层
  门控，建图因子可配置 Huber 鲁棒核，定位辅助按协方差自适应降低融合比例。质量差数据应由
  上游优先拒绝，LIS 会再次拒绝；禁止用零协方差表达“未知质量”。
- 先验地图加载会核对 manifest 版本、帧名、关键帧数量、PCD 数量与有限值，并清除点云中的
  NaN；损坏或与当前配置不兼容的地图启动即失败，不带病进入 ICP/因子图。
- 建图过程中任一逐关键帧 PCD/SCD 写入失败都会使本次地图失效；退出时不会生成或更新
  `map_manifest.yaml`。只有出现完整保存日志且清单存在时，地图才可交给定位模式。
- 没有 manifest 的历史地图按 legacy 模式兼容加载，并继续执行逐文件、点数和 Scan Context
  维度检查；该兼容规则不适用于本版本新产生但异常中断的地图。
- 建图输出目录必须为空；运行中使用 `.lvi_sam_mapping_in_progress` 标记事务未完成。
  异常退出后保留该标记，定位模式会拒绝加载，避免把新旧混合文件当成完整地图。
- schema 1 清单必须包含关键帧数量、Scan Context 维度和三个坐标帧字段；清单先写入临时文件，
  完整落盘后再替换正式文件，截断清单会在定位启动阶段被拒绝。

## 7. 扩展规则

1. 新增传感器先定义独立输入话题、消息类型、坐标系、时间基准、协方差语义和失效行为，
   再接入因子图。
2. 新增回环方法（如 Scan Context 变体或学习式描述子）应输出“候选 + 置信信息”，由统一
   几何验证层决定是否加入图优化，避免多个模块直接写因子图。
3. 先验地图格式升级必须增加版本字段并保留向后兼容读取；当前 schema 1 地图会核对
   manifest 与 PCD 数据完整性。相机重定位数据建议独立保存
   关键帧图像、时间戳、内外参版本、特征/描述子和对应三维点，不与 PCD 文件隐式绑定。
4. 新增 C++ 库应使用目标级 `target_link_libraries` / `ament_target_dependencies`，不得追加
   全局 ABI 路径或让一个进程加载两套 OpenCV。
5. 新配置必须通过 `validate_config.py`，新话题后缀应加入 `topic_names.hpp` 与接口测试。
6. 不得新增未登记的 `Odometry.covariance` 元数据槽位；内部契约升级必须同步修改
   `internal_odom_metadata.hpp`、生产者、消费者、本文档和回归测试。新设计优先使用具名消息字段。

## 8. Orin 验收门槛

提交合并前至少完成：

```bash
bash scripts/build.sh --clean
source install/setup.bash
colcon test --packages-select lvi_sam
colcon test-result --verbose

# 不应出现重复节点名；两个 LIS 内部参数上下文应各出现一次
ros2 node list | sort | uniq -d

ldd install/lvi_sam/lib/lvi_sam/visual_feature_node | grep -E 'opencv|cv_bridge'
ldd install/lvi_sam/lib/lvi_sam/visual_estimator_node | grep -E 'opencv|cv_bridge'
ldd install/lvi_sam/lib/lvi_sam/visual_loop_node | grep -E 'opencv|cv_bridge'
```

三个 VIS 进程不得出现 `libcv_bridge.so`，且每个进程只能解析到一套 OpenCV ABI。随后分别
完成静止、直线、转弯、闭环返回、定位丢失恢复、低质量 RTK 和相机断流测试；记录话题频率、
CPU/内存、轨迹连续性、回环接受/拒绝原因以及退出是否干净。
