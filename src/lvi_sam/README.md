# LVI-SAM-ROS2-Enhanced (`lvi_sam`)

> 路线 A1 落地工程：在现有 ROS2 LIO-SAM fork（激光 LIS）基础上，按 **LVI-SAM 原版架构**接入 **LVI-SAM-ROS2 的视觉子系统（VIS）**，形成"话题级松耦合"的激光-视觉-惯性 SLAM。
>
> 工程命名：`LVI-SAM-ROS2-Enhanced`（colcon 工作区）；主包名沿用 `lvi_sam`（与 LVI-SAM 原版一致，最小化 `PROJECT_NAME` / 话题字符串改动）。

---

## 1. 目录架构（参考 LVI-SAM 原版：配置外置 + 源码二分）

```
src/lvi_sam/
├── CMakeLists.txt          # 5 个 executable：LIS 2 + VIS 3
├── package.xml
├── config/                 # 【外层配置，集中摆放】
│   ├── params_mapping.yaml              # LIS 通用建图配置
│   ├── params_localization.yaml         # LIS 通用先验地图定位配置
│   ├── params_*_{localization,mapping}.yaml   # 场景/模式变体（gazebo/charging）
│   ├── params_camera.yaml               # VIS 参数（来自 LVI-SAM-ROS2）
│   ├── brief_k10L6.bin                  # DBoW2 词表（统一资源解析器加载）
│   ├── brief_pattern.yaml
│   ├── fisheye_mask_720x540.jpg
│   ├── rviz2.rviz
├── launch/
│   ├── run.launch.py                    # 总入口：启动 5 节点 + 接线 remap
│   └── include/module_sam_reference.py  # fork 原 launch 参考
├── include/                 # 公共头（激光 + 跨模块接口/资源适配）
│   ├── utility.hpp                     # ParamServer（激光参数集中声明）
│   ├── file_tools.hpp
│   └── sc/                             # Scan Context 库
├── scripts/                # gps_to_cartesian_node.py（已修硬编码路径）
└── src/
    ├── lidar_odometry/      # 【激光 LIS】来自 fork
    │   ├── imuPreintegration.cpp        # 节点：IMU 预积分 + TransformFusion
    │   ├── mapOptmization.cpp           # 节点：因子图 + 回环（SC/RTK/LoadPriorMap）
    │   ├── imageProjection.hpp          # 前端去畸变（内联类）
    │   └── featureExtraction.hpp        # LOAM 特征（内联类）
    └── visual_odometry/     # 【视觉 VIS】来自 LVI-SAM-ROS2（ROS2 化 VINS-Mono）
        ├── visual_feature/              # 节点1：光流跟踪 + camera_models
        ├── visual_estimator/            # 节点2：VINS 因子图 + factor/ initial/ feature_manager/ utility/
        └── visual_loop/                 # 节点3：DBoW2 回环 + ThirdParty/
```

源码严格二分：**`lidar_odometry/` = 激光**，**`visual_odometry/` = 视觉**；配置全部置于包根的 `config/`（外层集中，随 `install(DIRECTORY config)` 部署到 `share/lvi_sam/config`）。
配置选择、修改边界和预检命令见 [`config/README.md`](config/README.md)；完整改动记录见
根目录 [`docs/CHANGE_SUMMARY.md`](../../docs/CHANGE_SUMMARY.md)。

---

## 2. 节点与目标（CMakeLists 5 个 executable）

| 子系统 | executable | 源 |
|---|---|---|
| 激光 | `lvi_sam_imuPreintegration` | `src/lidar_odometry/imuPreintegration.cpp` |
| 激光 | `lvi_sam_mapOptimization` | `src/lidar_odometry/mapOptmization.cpp` + `include/sc/Scancontext.cpp` |
| 视觉 | `visual_feature_node` | `src/visual_odometry/visual_feature/*` + `camera_models/*.cc`（GLOB） |
| 视觉 | `visual_estimator_node` | `src/visual_odometry/visual_estimator/*.cpp`（GLOB） |
| 视觉 | `visual_loop_node` | `src/visual_odometry/visual_loop/*.cpp` + `ThirdParty/`（GLOB） |

---

## 3. VIS ↔ LIS 话题接线（松耦合，靠 remap + yaml 字段）

| 耦合点 | 方向 | 话题 | 接线方式 |
|---|---|---|---|
| ① 可选初始化先验 | LIS→VIS | `/odometry/imu` | launch 统一话题；`use_lidar_odometry_prior=1` 时 VIS 订阅 |
| ② 激光深度 | LIS→VIS | `lio_sam/deskew/cloud_deskewed` | `params_camera.yaml` 的 `point_cloud_topic` 已设为该绝对路径 |
| ③b 视觉回环候选 | VIS→LIS | VIS 发布 `/lvi_sam/vins/loop/match_frame`（Float64MultiArray=[cur_ts, old_ts]） | **launch remap**：mapOptimization 的 `lio_loop/loop_closure_detection` → `/lvi_sam/vins/loop/match_frame` |
| ③a 前端初值 | VIS→LIS | `/lvi_sam/vins/odometry/imu_propagate_ros` | fork 前端未订阅，**最小闭环暂不接**（可选增强） |

> VIS 内部话题（`/lvi_sam/vins/feature/feature`、`/restart`、`/odometry/...`）自动连通。
> `match_frame` 与 `detectLoopClosureExternal` 约定的 `data=[cur_ts, pre_ts]` 格式一致；
> LIS 会先执行时间容差映射，再通过点云 ICP 几何验证后才加入因子图。

---

## 4. 构建

```bash
# 依赖（Linux/ROS2 humble 环境）：
#   ament 系统包：rclcpp pcl_conversions tf2* visualization_msgs nav_msgs
#   原生依赖：GTSAM 4.x、Ceres、Boost、OpenCV、Eigen3、PCL
#   （已修正 LVI-SAM-ROS2 原版 Eigen3_DIR 硬编码 /opt/eigen；改用 find_package(Eigen3)）

cd LVI-SAM-ROS2-Enhanced
bash scripts/build.sh
source install/setup.bash
```

视觉节点通过 `include/lvi_sam/image_conversion.hpp` 完成 ROS 图像与 OpenCV 的转换，
不直接依赖 `cv_bridge`。输入支持 `rgb8`、`bgr8`、`rgba8`、`bgra8`、`mono8`
和 `8UC1`；未知编码、非法 `step` 或数据长度不足的图像会被安全丢弃并限频告警。

> ⚠️ 本工程在 Windows 下无法编译（ROS2 + VINS 依赖链需 Linux）。上述为在目标 Linux/ROS2 环境下的构建命令。

---

## 5. 运行

```bash
ros2 launch lvi_sam run.launch.py \
  lidar_params_file:=<path>/config/params_gazebo_localization.yaml \
  use_sim_time:=true \
  pcd_directory:=/tmp/lvi_sam_maps
```

常用参数：`lidar_params_file`（场景/模式 yaml）、`camera_params_file`、`pcd_directory`
（覆盖先验地图/输出目录）、`use_sim_time`、`project_name`、`odom_topic` 和
`publish_map_odom_static`。`imu_topic` 会同时覆盖 LIS 与 VIS 的标准
`sensor_msgs/Imu` 输入，默认 `/IMU_data`。

完整接口、依赖与稳定性约束见根目录
[`docs/INTERFACES_AND_STABILITY.md`](../../docs/INTERFACES_AND_STABILITY.md)。

---

## 6. 实机部署前必须确认

1. **IMU 接口**：LIS 与 VIS 现统一订阅 `/IMU_data`，类型为标准 `sensor_msgs/Imu`；若驱动实际发布其他话题或消息类型，须在驱动侧 remap/转换后再接入。
2. **相机-IMU-激光外参标定**：`params_camera.yaml` 里的 `extrinsicRotation/Translation`、`lidar_to_cam_*` 为示例值，须按实机标定填入（阶段 4）。
3. **时间同步**：图像/IMU/雷达时间戳对齐（MID360 已 IMU-雷达硬同步，相机需对齐）。
4. **先验地图/输出目录**：`pcd_directory` 默认 `/tmp/lvi_sam_maps`；实机请指向实际地图目录（launch 已覆盖 yaml 默认）。建图输出必须使用新目录或空目录，避免旧地图文件混入新地图。
5. **对称环境误闭环门控**：站场高度对称，建议给外部回环加 RTK/先验地图一致性门控（LVI-SAM 原版没有，可作创新点）。
6. **livox_ros_driver2 依赖包**：需放入本工作区 `src/`（或从系统/其它 workspace 提供），否则 LIS 无法编译。

---

## 7. 来源

- LIS：`return_sattion_ws_addgazebo_v0803/.../src/lio_sam`（ROS2 LIO-SAM fork，MID360/RTK/ScanContext/LoadPriorMap）。
- VIS：`多传感器融合slam/source/LVI-SAM-ROS2/lvi_sam`（ROS2 化 VINS-Mono 三节点）。
- 架构参考：`多传感器融合slam/source/LVI-SAM`（TixiaoShan 原版）。
