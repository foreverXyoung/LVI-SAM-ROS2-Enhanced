# 使用说明（USAGE）

本文档说明如何在已按 [ENVIRONMENT.md](ENVIRONMENT.md) 配好的环境中，**构建、配置并运行**
LVI-SAM-ROS2-Enhanced（LIS 激光 + VIS 视觉，话题级松耦合）。

---

## 1. 启动总览

`run.launch.py` 一次性拉起 **5 个节点**（LIS 2 + VIS 3）并完成双向话题接线：

```
激光驱动(livox_ros_driver2, 独立启动)
        │  /livox/lidar (+ /IMU_data)
        ▼
   LIS (imuPreintegration → mapOptimization)
        │  ├─ /lio_sam/odometry/imu ──────────► VIS.estimator      (① 位姿+尺度先验)
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
ros2 launch livox_ros_driver2 msg_MID360.launch.py
# 确认话题存在：
ros2 topic list | grep -E "/livox/lidar|/IMU_data"
```

> `/livox/lidar` 使用 `livox_ros_driver2/msg/CustomMsg`；LIS 与 VIS 默认共用
> `/IMU_data`，其类型必须为标准 `sensor_msgs/Imu`。

### 2.3 相机话题与内参

VIS 需要：
- 图像话题 `image_topic`（默认 `/camera/image_raw`，`sensor_msgs/Image`）。
- 相机内参写在 `config/params_camera.yaml` 的 `camera_intrinsics` / `distortion` 等字段，
  **须按实机标定填入**（仓库内为示例值）。

### 2.4 先验地图 / 输出目录

`pcd_directory` 既是读先验地图目录，也是建图输出目录。launch 默认 `/tmp/lvi_sam_maps`，
实机请指向实际路径（见 §3）。

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
| `imu_topic` | `/IMU_data` | LIS 与 VIS 共用的标准 `sensor_msgs/Imu` 话题；驱动发布 `/livox/imu` 时可在此统一覆盖 |
| `gps_topic` | 空 | 可选 map 对齐 `nav_msgs/Odometry` RTK/GPS 话题；非空时覆盖 YAML |
| `enable_visual` | `true` | 无相机或仅验证激光链路时设为 `false` |
| `pcd_directory` | `/tmp/lvi_sam_maps` | 先验地图读取 / 建图输出目录，**覆盖 yaml 内默认值** |
| `use_sim_time` | `false` | 仿真（Gazebo）置 `true` |
| `publish_map_odom_static` | `false` | 是否发布 `map→odom` 静态变换；仅在确认没有其他 TF 发布者时启用 |

> 也可用 [`scripts/run.sh`](../scripts/run.sh) 透传参数：
> `bash scripts/run.sh robot_description_file:=/path/to/robot.urdf.xacro use_sim_time:=true`

---

## 4. 话题接线核验

启动后逐项确认耦合是否接通（任一不通都会让对应子系统退化）：

```bash
# ① LIS→VIS 位姿/尺度先验：fork 与 estimator 同在 /odometry/imu
ros2 topic echo /odometry/imu --field pose.pose.position -n 1

# ② LIS→VIS 激光深度
ros2 topic hz /lio_sam/deskew/cloud_deskewed

# ③b VIS→LIS 视觉回环候选（仅在 VIS 检测到回环时才发布）
ros2 topic echo /lvi_sam/vins/loop/match_frame -n 1

# 四大对外/topic 是否正常
ros2 topic hz /lio_sam/mapping/odometry
ros2 topic hz /lio_sam/mapping/cloud_registered
```

### 关键话题一览

| 话题 | 类型 | 方向 / 说明 |
|------|------|------|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | LIS 输入（原始点云） |
| `/IMU_data` | `sensor_msgs/Imu` | LIS 与 VIS 共用的标定后 IMU |
| `/odometry/imu` | `nav_msgs/Odometry` | **LIS→VIS** 位姿+尺度先验 |
| `/lio_sam/deskew/cloud_deskewed` | `sensor_msgs/PointCloud2` | **LIS→VIS** 激光去畸变深度 |
| `/lvi_sam/vins/loop/match_frame` | `std_msgs/Float64MultiArray` | **VIS→LIS** 视觉回环候选 `[cur_ts, old_ts]` |
| `/lio_sam/mapping/odometry` | `nav_msgs/Odometry` | LIS 对外里程计 |
| `/lio_sam/mapping/cloud_registered` | `sensor_msgs/PointCloud2` | LIS 建图点云 |

> ⚠️ **不要**手动给 `odometry/imu` 加 remap 到 `/lio_sam/odometry/imu`：
> fork 的 `odomTopic` 在 `params.yaml` 中被覆盖为相对 `"odometry/imu"`，
> 与 estimator 订阅的相对话题解析为同一 `/odometry/imu`，remap 反而会让 VIS 收不到先验。

---

## 5. 参数文件说明

### 5.1 `config/params.yaml`（LIS）

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
- `imu_topic: /IMU_data`、`image_topic: /camera/image_raw`。
- `point_cloud_topic: /lio_sam/deskew/cloud_deskewed`（② 激光深度，绝对路径）。
- `camera_intrinsics` / `distortion`：相机内参（**须标定**）。
- `extrinsicRotation` / `extrinsicTranslation`：相机→IMU 外参（**须标定**）。
- `vocabulary_file` / `brief_pattern_file` / `fisheye_mask`：DBoW2 词表/模板/掩膜，
  代码按 `pkg_path + "/config/"` 拼接，已置于 `config/` 根，**不要移动**。

---

## 6. 最小验证闭环（建议先跑）

目标：在**不标定外参、不接真实相机**的情况下，先确认管线能跑通。

1. 用仿真或录制的 bag 提供 `/livox/lidar`、`/IMU_data`、`/camera/image_raw`。
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
| VIS 完全不动 / 无位姿 | `ros2 topic hz /IMU_data`、`/camera/image_raw`、`/odometry/imu` 是否都有数据 |
| VIS 尺度 drift / 飞掉 | 确认 `odometry/imu` 接通（① 先验）；单目 VIS 离了 LIS 尺度会失控 |
| LIS 收不到视觉回环 | `ros2 topic echo /lvi_sam/vins/loop/match_frame`；确认 remap 未被改动 |
| LIS 不发布 `cloud_deskewed` | 检查 `params.yaml` 的 `deskew` 开关与输入点云时间戳 |
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
