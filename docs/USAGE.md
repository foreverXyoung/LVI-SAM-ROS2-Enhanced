# 使用说明（USAGE）

本文档说明如何在已按 [ENVIRONMENT.md](ENVIRONMENT.md) 配好的环境中，**构建、配置并运行**
LVI-SAM-ROS2-Enhanced（LIS 激光 + VIS 视觉，话题级松耦合）。

---

## 1. 启动总览

`run.launch.py` 一次性拉起 **5 个算法节点**（LIS 2 + VIS 3）并完成双向话题接线；
RViz2 默认作为第 6 个进程启动：

```
激光驱动(livox_ros_driver2, 独立启动)
        │  /livox/lidar (+ /IMU_data)
        ▼
   LIS (imuPreintegration → mapOptimization)
        │  ├─ /odometry/imu ───────────────────► VIS.estimator      (① 可选初始化先验)
        │  └─ /lio_sam/deskew/cloud_deskewed ─► VIS.feature_tracker (② 激光深度)
        ▼
   VIS (feature → estimator → loop)
        │  /lvi_sam/vins/loop/match_frame ───► LIS.mapOptimization (③b 视觉回环候选)
        ▼
   对外：/lio_sam/mapping/odometry, /lio_sam/mapping/cloud_registered
```

---

## 2. 启动前准备（必须）

### 2.1 机器人 URDF / TF 链

LIS 与 VIS 都依赖正确的 TF。必须提供包含以下链路的 URDF（xacro）：
`map → odom → base_link → livox_frame`（激光）、`base_link → imu_link`（IMU）、
以及相机帧（如 `base_link → camera_link → camera`）。

- 通过 `robot_description_file` 参数传入，launch 会启动 `robot_state_publisher`。
- 也可由你自己的 launch 提供 `robot_description` + `robot_state_publisher`，本 launch 的对应节点可关掉（见 §3 参数）。

> 若只是想先验证 VIS 出位姿，可临时用一个最小 URDF（仅 base_link + 各传感器静态 TF）。

### 2.2 激光驱动（livox_ros_driver2）

**本 launch 不启动激光驱动**，需单独启动（不同平台 launch 不同）：

```bash
# MID360 示例
ros2 launch livox_ros_driver2 msg_MID360_launch.py
# 确认话题存在：
ros2 topic list | grep -E "/livox/lidar|/IMU_data"
```

> `/livox/lidar` 使用 `livox_ros_driver2/msg/CustomMsg`；LIS 与 VIS 默认共用
> `/IMU_data`，其类型必须为标准 `sensor_msgs/Imu`。

### 2.3 相机话题与内参

VIS 需要：
- 图像话题 `image_topic`（默认 `/camera/color/image_raw`，`sensor_msgs/Image`）。当前实机输出
  `rgb8`；内部适配层也支持 `bgr8`、`rgba8`、`bgra8`、`mono8` 和 `8UC1`。
- 相机内参写在 `config/params_camera.yaml` 的 `projection_parameters` /
  `distortion_parameters` 字段，
  **须按实机标定填入**（仓库内为示例值）。

### 2.4 先验地图 / 输出目录

`pcd_directory` 既是读先验地图目录，也是建图输出目录。launch 默认 `/tmp/lvi_sam_maps`，
实机请指向实际路径（见 §3）。建图模式要求该目录为新目录或空目录；不要覆盖已有地图。
运行期间会创建 `.lvi_sam_mapping_in_progress`，只有 Ctrl+C 正常退出且全部地图文件写入成功后
才会删除该标记并提交 `map_manifest.yaml`。

---

## 3. 启动命令与参数

### 3.1 最简启动（仿真/已具备 URDF）

```bash
source install/setup.bash
ros2 launch lvi_sam run.launch.py \
  mode:=mapping scene:=generic \
  robot_description_file:=/path/to/your_robot.urdf.xacro \
  use_sim_time:=true \
  pcd_directory:=/tmp/lvi_sam_maps
```

### 3.2 实机启动（指定场景参数）

```bash
ros2 launch lvi_sam run.launch.py \
  mode:=localization scene:=charging \
  camera_params_file:=$(ros2 pkg prefix lvi_sam)/share/lvi_sam/config/params_camera.yaml \
  pcd_directory:=/home/$USER/maps \
  gps_topic:=/gps/lio_sam_odom \
  use_sim_time:=false
```

### 3.3 launch 参数表

| 参数 | 默认 | 说明 |
|------|------|------|
| `mode` | `mapping` | `mapping` 或 `localization` |
| `scene` | `generic` | `generic`、`charging` 或 `gazebo`；与 `mode` 组合选择 YAML |
| `lidar_params_file` | 空 | 可选显式 LIS 参数文件；非空时优先于 `mode/scene` |
| `camera_params_file` | `config/params_camera.yaml` | VIS 参数 |
| `image_topic` | `/camera/color/image_raw` | VIS 输入图像话题；覆盖相机参数文件中的同名配置 |
| `imu_topic` | `/IMU_data` | LIS 与 VIS 共用的标准 `sensor_msgs/Imu` 话题；驱动发布 `/livox/imu` 时可在此统一覆盖 |
| `odom_topic` | `/odometry/imu` | LIS IMU 预积分输出与 VIS 位姿/尺度先验输入；入口统一覆盖两侧配置 |
| `project_name` | `lvi_sam` | 仅作为 VIS 话题根；所有视觉内部接口发布到 `/<project_name>/vins/...` |
| `gps_topic` | 空 | 可选 map 对齐 `nav_msgs/Odometry` RTK/GPS 话题；非空时覆盖 YAML |
| `enable_visual` | `true` | 无相机或仅验证激光链路时设为 `false` |
| `enable_rviz` | `true` | 默认启动工程 RViz2；无桌面或纯 SSH 环境设为 `false` |
| `rviz_config_file` | `config/rviz2.rviz` | RViz2 配置文件 |
| `rviz_fixed_frame` | `odom` | RViz2 Fixed Frame；当前实机算法输出默认使用 `odom` |
| `pcd_directory` | `/tmp/lvi_sam_maps` | 先验地图读取 / 建图输出目录，**覆盖 yaml 内默认值** |
| `use_sim_time` | `false` | 仿真（Gazebo）置 `true` |
| `publish_map_odom_static` | `false` | 是否发布 `map→odom` 静态变换；仅在确认没有其他 TF 发布者时启用 |
| `publish_fused_tf` | `true` | 是否发布 `odom→base_link`；纯算法测试设为 `false`，此时不会查询平台 TF tree |

> 也可用 [`scripts/run.sh`](../scripts/run.sh) 透传参数：
> `bash scripts/run.sh robot_description_file:=/path/to/robot.urdf.xacro use_sim_time:=true`

---

## 4. 话题接线核验

启动后逐项确认耦合是否接通（任一不通都会让对应子系统退化）：

```bash
# ① LIS→VIS 初始化先验输入；只有 use_lidar_odometry_prior=1 时 VIS 才订阅
ros2 topic echo /odometry/imu --once --field pose.pose.position

# ② LIS→VIS 激光深度
ros2 topic hz /lio_sam/deskew/cloud_deskewed

# ③b VIS→LIS 视觉回环候选（仅在 VIS 检测到回环时才发布）
ros2 topic echo /lvi_sam/vins/loop/match_frame --once

# 四大对外/topic 是否正常
ros2 topic hz /lio_sam/mapping/odometry
ros2 topic hz /lio_sam/mapping/cloud_registered
```

### 关键话题一览

| 话题 | 类型 | 方向 / 说明 |
|------|------|------|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | LIS 输入（原始点云） |
| `/IMU_data` | `sensor_msgs/Imu` | LIS 与 VIS 共用的标定后 IMU |
| `/odometry/imu` | `nav_msgs/Odometry` | **LIS→VIS** 可选位姿/速度/尺度初始化先验 |
| `/lio_sam/deskew/cloud_deskewed` | `sensor_msgs/PointCloud2` | **LIS→VIS** 激光去畸变深度 |
| `/lvi_sam/vins/loop/match_frame` | `std_msgs/Float64MultiArray` | **VIS→LIS** 视觉回环候选 `[cur_ts, old_ts]` |
| `/lio_sam/mapping/odometry` | `nav_msgs/Odometry` | LIS 对外里程计 |
| `/lio_sam/mapping/cloud_registered` | `sensor_msgs/PointCloud2` | LIS 建图点云 |

> `odom_topic` 会同时覆盖 LIS 的 `odomTopic` 与 VIS 的 `odom_topic`。不要只 remap
> 其中一侧，否则会切断位姿/尺度先验。默认统一使用绝对话题 `/odometry/imu`。

---

## 5. 参数文件说明

### 5.1 `config/params_mapping.yaml` / `params_localization.yaml`（LIS）

关键字段（节选，按子系统分组）：

- **硬件 / 外参**：`lidarFrame: livox_frame`、`baselinkFrame: base_link`、
  `extrinsicRot` / `extrinsicRPY`（livox→base 外参）。
- **里程计**：`odomTopic`（已被覆盖为 `odometry/imu`）、`imuTopic: /IMU_data`、
  `pointCloudTopic: /livox/lidar`。
- **回环**：`loopClosureEnableFlag` 控制回环线程，`scanContextLoopEnableFlag`
  控制在线 Scan Context 候选，`scanContextDistanceThreshold` 控制候选相似度门限；
  视觉和 Scan Context 候选最终都由 ICP 验证。
- **地图**：`savePCD`、`savePCDDirectory`、`Loc.loadPCDDirectory`（先验地图）。
- **RTK / 重定位**：建图使用显式 `useGpsFactor`；定位使用独立的
  `Loc.useRTKAssist` / `Loc.useRTKInitialization`。建议同时配置
  `gpsExpectedFrame`（建图）或 `Loc.rtkExpectedFrame`（定位）以拒绝坐标系错误的数据；
  融合还带有协方差、时间、连续稳定样本和创新量门控。

场景变体：`params_gazebo_*.yaml`（仿真）、`params_charging_*.yaml`（充电场景）、
`*_localization.yaml` / `*_mapping.yaml`（定位/建图模式）。通常通过 `mode` 和 `scene`
切换；需要自定义文件时再使用 `lidar_params_file`。

### 5.2 `config/params_camera.yaml`（VIS）

关键字段：

- `PROJECT_NAME: lvi_sam` → 决定 VIS 发布话题前缀（`/lvi_sam/vins/...`）。
- `imu_topic: /IMU_data`、`image_topic: /camera/color/image_raw`。
- `point_cloud_topic: /lio_sam/deskew/cloud_deskewed`（② 激光深度，绝对路径）。
- `use_lidar` 与 `use_lidar_odometry_prior` 分别控制激光深度投影和 LIS 里程计
  初始化先验；未完成 LiDAR-camera 标定时两者均保持 `0`，VIS 节点仍会运行。
- `camera_intrinsics` / `distortion`：相机内参（**须标定**）。
- `extrinsicRotation` / `extrinsicTranslation`：相机→IMU 外参（**须标定**）。
- `vocabulary_file` / `brief_pattern_file` / `fisheye_mask`：DBoW2 词表/模板/掩膜，
  可写绝对文件路径，也可写相对 `share/lvi_sam/` 的包内路径（默认位于 `config/`）。
  启动时会解析并检查文件存在性；不要依赖当前工作目录。

---

## 6. 最小验证闭环（建议先跑）

目标：在**不标定外参、不接真实相机**的情况下，先确认管线能跑通。

1. 用仿真或录制的 bag 提供 `/livox/lidar`、`/IMU_data`、`/camera/color/image_raw`。
2. 启动 `run.launch.py`（`use_sim_time:=true`），观察：
   - `visual_estimator_node` 是否输出 `/lvi_sam/vins/odometry/...`（VIS 位姿）。
   - `visual_loop_node` 是否在经过相似场景时发布 `/lvi_sam/vins/loop/match_frame`。
   - LIS `/lio_sam/mapping/odometry` 是否平滑、无发散。
3. 若 VIS 回环被 LIS 接受：`ros2 topic echo /lio_sam/mapping/odometry` 在回环处应出现位姿跳变修正。

通过后再推进：外参标定（§2.2/§5.2）、时间同步、站场实测与误闭环门控。

---

## 7. 排错

| 现象 | 排查 |
|------|------|
| VIS 节点起不来 / 闪退 | 检查 `params_camera.yaml` 是否存在、`vocabulary_file` 路径；看节点 stderr 是否报「文件不存在」 |
| VIS 完全不动 / 无位姿 | 先检查 `/IMU_data`、`/camera/color/image_raw` 和 `/lvi_sam/vins/feature/feature`；仅当 `use_lidar_odometry_prior=1` 时才要求 `/odometry/imu` |
| VIS 尺度 drift / 飞掉 | 优先核对相机–IMU 标定、IMU 噪声和图像–IMU 时间同步；完成 LiDAR–camera 标定后可再启用 `/odometry/imu` 初始化先验 |
| LIS 收不到视觉回环 | `ros2 topic echo /lvi_sam/vins/loop/match_frame`；确认 remap 未被改动 |
| LIS 不发布 `cloud_deskewed` | 检查当前模式的 LIS 参数文件以及输入点云、IMU 时间戳 |
| TF 报错 `base_link` 缺失 | URDF 未提供或未启动 `robot_state_publisher` |
| 激光驱动话题名不符 | 不同 livox launch 话题名可能不同，按实机调整 `pointCloudTopic` / `imuTopic` |

---

## 8. 调参与后续

- **对称/长直场景误闭环**：站场高度对称，Scan Context 与视觉回环都可能误匹配；
  当前候选会经过 ICP/适应度验证，实机仍应结合 RTK 创新量和连续轨迹一致性调门限。
- **视觉跨会话重定位**：当前 DBoW2 数据库仍是内存数据库；需要持久化的数据集和
  激光二次验证链路见 `ARCHITECTURE_AND_MAP_FORMAT.md`。
- **VIS→LIS 前端初值（③a）**：`imu_propagate_ros` 暂未接入（最小闭环不需要），可作为精度增强项后续补。
- **多场景切换**：使用 `mode:=mapping|localization` 与
  `scene:=generic|gazebo|charging`，无需手写配置文件绝对路径。
