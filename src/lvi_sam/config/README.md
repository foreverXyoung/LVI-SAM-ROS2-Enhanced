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
| `params_camera.yaml` | VIS 相机、IMU、LiDAR-camera、视觉回环参数 | 完成标定后修改 |
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
  enable_visual:=false enable_rviz:=false \
  pcd_directory:=/data/return_station_ws/maps/lidar_check_001
```

只有临时试验或外部配置管理器确实需要时，才使用绝对路径覆盖：

```bash
ros2 launch lvi_sam run.launch.py \
  lidar_params_file:=/absolute/path/lidar.yaml \
  camera_params_file:=/absolute/path/camera.yaml
```

参数优先级为：**launch 显式覆盖 > 选中的 YAML > C++ 声明默认值**。`imu_topic`、
`odom_topic`、`image_topic`、`gps_topic` 和 `pcd_directory` 属于部署级参数，优先在 launch
命令中覆盖；算法阈值和标定值留在 YAML 中。

注意两个名称相近但作用不同的开关：launch 的 `enable_rviz` 控制是否启动 RViz2 进程；LIS
YAML 顶层的 `useRviz` 控制是否发布较重的点云/轨迹可视化数据。`useRviz` 不能缩进到 `Loc`。

## 3. 修改时按组处理

### 3.1 传感器与坐标接口

同一 `scene` 的 mapping/localization 文件必须同时修改以下字段：

- `pointCloudTopic`、`imuTopic`、`odomTopic`；
- `lidarFrame`、`baselinkFrame`、`odometryFrame`、`mapFrame`；
- `sensor`、`N_SCAN`、`Horizon_SCAN`、`downsampleRate`；
- `extrinsicRot`、`extrinsicRPY`、`extrinsicTrans`。

这些字段决定输入解释和地图坐标契约，建图与定位不一致时禁止复用地图。配置预检会自动拒绝
同一场景中不一致的配置。

### 3.2 三组外参不能混用

- LIS YAML 的 `extrinsicRot` / `extrinsicRPY` / `extrinsicTrans`：IMU 与 LiDAR；
- camera YAML 的 `extrinsicRotation` / `extrinsicTranslation`：camera 与 IMU；
- camera YAML 的 `lidar_to_cam_*`：LiDAR 与视觉虚拟深度帧。

标定完成前保持 `use_lidar: 0`、`use_lidar_odometry_prior: 0`、
`align_camera_lidar_estimation: 0`。建议依次启用视觉单目、LIS 里程计先验、LiDAR 深度，
每次只增加一条耦合链路并保存对应测试日志。

### 3.3 建图、定位和 RTK

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

预检覆盖 YAML 重复键、六套活动 LIS 配置、旧兼容配置、同场景建图/定位传感器一致性、旋转矩阵、数值范围、
LIS/VIS 话题一致性和视觉资源完整性。修改源码目录后，非 `--symlink-install` 工作区需要重新
构建；不要直接编辑 `install/lvi_sam/share/lvi_sam/config`，因为下次构建会覆盖它。
