# 配置入口与修改规则

本目录是 `lvi_sam` 的唯一配置源。新部署应先在这里选择场景和模式，再通过
`run.launch.py` 启动；不要在 C++ 源码、launch 文件或 `install/` 目录中复制参数。

## 1. 配置选择矩阵

| `scene` | `mode=mapping` | `mode=localization` | 用途 |
|---|---|---|---|
| `generic` | `params_mapping.yaml` | `params_localization.yaml` | 当前 Mid-360 实机通用配置 |
| `charging` | `params_charging_mapping.yaml` | `params_charging_localization.yaml` | 回站/充电场景，含按需启用的 RTK 配置 |
| `gazebo` | `params_gazebo_mapping.yaml` | `params_gazebo_localization.yaml` | 仿真传感器和仿真外参 |

其他文件：

| 文件 | 作用 | 是否直接修改 |
|---|---|---|
| `params_imu_external.yaml` | 外置标定 IMU：话题、单位、噪声和 IMU-LiDAR 外参 | 外置 IMU 重新标定后修改 |
| `params_imu_mid360.yaml` | MID-360 内置 IMU：话题、`g→m/s²`、噪声和同轴外参 | Allan 标定/杆臂测量后修改 |
| `params_mount_robot.yaml` | 机器人 `base_link→LiDAR` 安装位姿，与所选 IMU 无关 | 雷达重新安装或 base 基准改变后修改 |
| `params_camera.yaml` | 外置 IMU 的 VIS 相机、外参和视觉回环参数 | 完成标定后修改 |
| `params_camera_mid360.yaml` | MID-360 内置 IMU、实机 `T_cam_radar` 与视觉参数 | 相机或雷达重装、重新标定后修改 |
| `rviz2.rviz` | 默认 RViz 显示项 | 可按显示需求修改 |
| `brief_k10L6.bin` / `brief_pattern.yaml` | DBoW2/BRIEF 资源 | 一般不修改 |
| `fisheye_mask_720x540.jpg` | 仅鱼眼模式使用 | `fisheye=0` 时不加载 |
| `params.yaml` | 旧部署兼容定位配置 | 新部署不要使用 |

`params.yaml` 不会被 `run.launch.py` 自动选择。保留它只是为了兼容旧脚本和已经复制该文件的
部署；配置预检会要求它的传感器接口和通用定位配置保持一致。

## 2. 推荐启动方式

```bash
# 实机建图
ros2 launch lvi_sam run.launch.py \
  scene:=generic mode:=mapping \
  pcd_directory:=/data/return_station_ws/maps/site_001

# 使用上一步完整地图定位
ros2 launch lvi_sam run.launch.py \
  scene:=generic mode:=localization \
  pcd_directory:=/data/return_station_ws/maps/site_001

# 无桌面或纯激光排障
ros2 launch lvi_sam run.launch.py \
  scene:=generic mode:=mapping \
  imu_source:=external \
  enable_visual:=false enable_rviz:=false \
  pcd_directory:=/data/return_station_ws/maps/lidar_check_001

# MID-360 内置 IMU 对比测试（先只测试 LIS）
ros2 launch lvi_sam run.launch.py \
  scene:=generic mode:=mapping \
  imu_source:=mid360 \
  enable_visual:=false \
  pcd_directory:=/data/return_station_ws/maps/mid360_imu_001

# MID-360 内置 IMU + 相机联调（自动选择 params_camera_mid360.yaml）
ros2 launch lvi_sam run.launch.py \
  scene:=generic mode:=mapping imu_source:=mid360 \
  enable_visual:=true use_sim_time:=false \
  pcd_directory:=/data/return_station_ws/maps/mid360_visual_001
```

只有临时试验或外部配置管理器确实需要时，才使用绝对路径覆盖：

```bash
ros2 launch lvi_sam run.launch.py \
  lidar_params_file:=/absolute/path/lidar.yaml \
  imu_params_file:=/absolute/path/new_imu.yaml \
  camera_params_file:=/absolute/path/camera.yaml \
  mount_params_file:=/absolute/path/base_to_lidar.yaml
```

参数优先级为：**launch 显式覆盖 > 安装/IMU profile > 场景/模式 YAML > C++ 声明默认值**。
正常切换 IMU 使用 `imu_source:=external|mid360`，不要只改 `imu_topic`；后者仅用于临时话题
重映射，不会改变外参、噪声或单位。`odom_topic`、`image_topic`、`gps_topic` 和
`pcd_directory` 属于部署级参数；算法阈值和标定值留在 YAML 中。
接入第三种 IMU 时复制现有 profile，完整填写同一组字段，然后通过绝对路径
`imu_params_file:=/path/to/params_imu_new.yaml` 加载；不需要修改 launch 或 C++。

`localization_reset_topic` 是跨节点复位事件的部署级话题，默认
`/lio_sam/localization/reset`。通常不需要修改；只有同一 ROS graph 中运行多套
LVI-SAM 实例时，才为每套实例指定不同的绝对话题。它不是 Nav2 的速度或 TF 指令，
上层应优先消费结构化 `/lio_sam/localization/status`，并按事件中的动作标志进行协同。

注意两个名称相近但作用不同的开关：launch 的 `enable_rviz` 控制是否启动 RViz2 进程；LIS
YAML 顶层的 `useRviz` 控制是否发布较重的点云/轨迹可视化数据。`useRviz` 不能缩进到 `Loc`。

## 3. 修改时按组处理

### 3.1 传感器与坐标接口

同一 `scene` 的 mapping/localization 文件必须同时修改以下字段：

- `pointCloudTopic`、`imuTopic`、`odomTopic`；
- `lidarFrame`、`baselinkFrame`、`odometryFrame`、`mapFrame`；
- `sensor`、`N_SCAN`、`Horizon_SCAN`、`downsampleRate`。

这些字段决定输入解释和地图坐标契约，建图与定位不一致时禁止复用地图。配置预检会自动拒绝
同一场景中不一致的配置。

IMU 是独立的配置维度：

- `imu_source:=external` 加载 `params_imu_external.yaml`，默认 `/IMU_data`，加速度比例为 `1.0`；
- `imu_source:=mid360` 加载 `params_imu_mid360.yaml`，默认 `/livox/imu`，将 Livox 原始
  `g` 单位乘以 `9.80665` 后再送入 LIS/VIS；
- 两份 profile 必须将话题、噪声、随机游走、重力、姿态权重和 IMU-LiDAR 外参一起修改。
  禁止仅更换话题；这会把一种 IMU 的数据按另一种 IMU 的标定解释。

IMU profile 使用三个有方向含义的标定字段：

- `imuToLidarRotation`：原始 IMU 向量到 LiDAR 轴向，满足
  `v_lidar = R_lidar_imu · v_imu`；
- `imuToLidarTranslation`：LiDAR 原点在原始 IMU 坐标系中的位置，单位 m；
- `imuOrientationToLidarRotation`：姿态消息右乘修正，满足
  `q_world_lidar = q_world_imu · q_imu_lidar`。

`imuOrientationSource=message` 表示使用 IMU 姿态消息；`mount` 表示驱动没有可用姿态，
启动时使用安装 profile 的 `baseToLidarRotation`，并假设机器人启动时处于水平静止状态。

MID-360 驱动不提供可用姿态估计，因此 profile 将 `imuRPYWeight` 设为 `0.0`，但仍使用角速度
进行点云去畸变。内置 IMU 与点云采用单位旋转；平移初值采用
[FAST-LIO 官方 Mid-360 配置](https://github.com/hku-mars/FAST_LIO/blob/main/config/mid360.yaml)
中的 `[-0.011, -0.02329, 0.04412] m`。该值是官方参考初值，若获得当前设备的实测标定结果，
应优先覆盖本 profile。`imu_source:=mid360` 会同时自动选择
`params_camera_mid360.yaml`。当前文件已经把实机标定的
`p_camera_optical = T_cam_radar · p_lidar` 转换为算法使用的 ROS 前左上虚拟深度帧外参，
并启用激光深度。相机—IMU 外参由该标定与 FAST-LIO 内置 IMU 杆臂初值组合，仍保持在线细化；
LIS 里程计先验和全局视觉—激光对齐暂时关闭，以便分阶段排查时间同步与坐标方向。

标定文件只提供 `K`、未提供畸变向量，因此 `fx/fy/cx/cy` 已更新，`k1/k2/p1/p2` 暂时保留
此前 CameraInfo 数值。若最新 `/camera/color/camera_info` 的 `d` 不同，应同步替换。

### 3.2 四组外参不能混用

- IMU profile 的 `imuToLidar*`：所选 IMU 与 LiDAR，移动/更换 IMU 时修改；
- `params_mount_robot.yaml` 的 `baseToLidar*`：机器人本体与 LiDAR，重新安装雷达时修改；
- camera YAML 的 `extrinsicRotation` / `extrinsicTranslation`：camera 与 IMU；
- camera YAML 的 `lidar_to_cam_*`：LiDAR 与视觉虚拟深度帧。

旧 `extrinsicRot` / `extrinsicRPY` / `extrinsicTrans` 仍可被 C++ 读取作为兼容回退，但新部署
不得继续新增这组模糊参数。加载了安装 profile 后，融合节点直接用 `T_base_lidar` 计算
`odom→base_link`，不会查询 TF tree。若启用 `publish_fused_tf` 且
`lidarFrame != baselinkFrame`，缺少 `baseToLidar*` 会立即报错，不再用 TF 或单位矩阵猜测安装关系。

所有物理外参都遵循同一规则：源码只读取、校验并组合参数，不保存任何具体设备的平移或角度。
更换传感器时只需复制/修改对应 profile：

1. 更换 IMU：新增 `params_imu_<name>.yaml`，填写 `imuToLidar*`、噪声、单位和话题；
2. 移动 LiDAR：修改独立的 `params_mount_robot.yaml`，同时重新测量本体过滤盒；
3. 更换/移动相机：复制 camera profile，填写内参、`extrinsicRotation/Translation` 和
   `lidar_to_cam_*`；
4. 使用第三种 IMU 时通过 `imu_params_file` 加载，并显式提供匹配的 `camera_params_file`。

启用某条融合链路但遗漏对应外参时，节点会在启动阶段失败。代码不再以零平移、单位旋转或
TF 查询作为物理外参兜底。

标定完成前保持 `use_lidar: 0`、`use_lidar_odometry_prior: 0`、
`align_camera_lidar_estimation: 0`。建议依次启用视觉单目、LIS 里程计先验、LiDAR 深度，
每次只增加一条耦合链路并保存对应测试日志。

### 3.3 本体点云过滤

`selfFilterEnable` 控制原始点云的轴对齐包围盒过滤，过滤发生在去畸变之前：

- `selfFilterFrame: lidar`：盒子直接用 `livox_frame` 表达，不依赖 TF，适合沿用已经调过的盒子；
- `selfFilterFrame: base`：先用 `params_mount_robot.yaml` 的 `T_base_lidar` 把点变到
  `base_link` 再判断，仍不查询 TF tree，适合按机器人本体尺寸配置；
- `selfFilterBoxMin/Max` 必须逐轴满足 `min < max`。盒子过小会留下本体，过大会误删近距离环境。

通用实机 profile 现在启用原有的 LiDAR-frame 小盒；应在 RViz 中观察静态本体残点后再逐轴调整。
更换 IMU 不影响该盒；雷达安装位置改变时需要重新测量。

### 3.4 建图、定位和 RTK

- mapping：`Loc.EnableFlag=false`、`savePCD=true`；输出目录必须是新目录或空目录。
- localization：`Loc.EnableFlag=true`、`savePCD=false`；本版本新建地图必须包含完整
  `map_manifest.yaml`，且不能保留写入中标记。没有清单的历史地图按 legacy 模式兼容加载，
  但仍执行逐文件、点数和 Scan Context 维度检查。
- RTK 默认关闭。建图因子由 `useGpsFactor` 控制，定位辅助由
  `Loc.useRTKAssist` 控制；输入必须是地图坐标系下的 `nav_msgs/Odometry`，不能直接把
  `NavSatFix` 当作地图坐标使用。
- RTK 上游应提供真实协方差并完成投影、质量判断和杆臂补偿；零协方差不能表示“质量很好”。

## 4. 默认数据接口

| 方向 | 话题 | 类型 |
|---|---|---|
| 输入 | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` |
| 输入 | `/IMU_data` | `sensor_msgs/msg/Imu` |
| 可选输入 | `/livox/imu` | `sensor_msgs/msg/Imu`（原始加速度为 `g`，由 profile 转换） |
| 输入 | `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| LIS 输出 | `/lio_sam/mapping/odometry` | `nav_msgs/msg/Odometry` |
| LIS→VIS | `/odometry/imu` | `nav_msgs/msg/Odometry`（含内部兼容元数据） |
| LIS→VIS | `/lio_sam/deskew/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` |
| VIS→LIS | `/lvi_sam/vins/loop/match_frame` | `std_msgs/msg/Float64MultiArray` |

完整 QoS、坐标系、时间和内部元数据语义见
[`../../../docs/INTERFACES_AND_STABILITY.md`](../../../docs/INTERFACES_AND_STABILITY.md)。

## 5. 每次修改后的强制预检

```bash
python3 src/lvi_sam/scripts/validate_config.py \
  --config-dir src/lvi_sam/config

# 纯激光构建时
python3 src/lvi_sam/scripts/validate_config.py \
  --config-dir src/lvi_sam/config --lidar-only
```

预检覆盖 YAML 重复键、六套活动 LIS 配置、两套 IMU profile、旧兼容配置、同场景建图/定位
传感器一致性、旋转矩阵、数值范围、LIS/VIS 话题一致性和视觉资源完整性。修改源码目录后，非 `--symlink-install` 工作区需要重新
构建；不要直接编辑 `install/lvi_sam/share/lvi_sam/config`，因为下次构建会覆盖它。
