# 架构、地图格式与多源重定位约定

## 1. 运行职责

- `imuPreintegration`：IMU 预积分及高频里程计，不拥有 `map` 坐标系。
- `mapOptimization`：激光建图、先验地图定位、Scan Context、ICP、位姿图及 RTK/GPS 约束，是地图状态的唯一拥有者。
- `visual_feature`：图像跟踪与激光深度注册，不保存全局地图。
- `visual_estimator`：滑窗视觉惯性估计，为视觉回环提供关键帧位姿和三维点。
- `visual_loop`：DBoW2/BRIEF 候选检索和 PnP 几何验证。视觉结果只是候选约束，最终仍须由 `mapOptimization` 结合激光/先验地图验证。

视觉进程通过 `include/lvi_sam/image_conversion.hpp` 将标准 `sensor_msgs/Image` 转为持有
自身内存的 OpenCV 灰度图。该适配层替代对预编译 `cv_bridge` 的直接链接，使每个 LVI-SAM
进程只加载 CMake 选中的一套 OpenCV；它只改变消息适配，不改变光流、VINS 或回环算法。

TF 的唯一发布原则：机器人主系统或 TransformFusion 负责 `odom→base_link`；只有在不存在其他发布者时，才允许入口 launch 发布静态 `map→odom`。

## 2. 激光地图格式

新建地图目录至少包含：

```text
map_directory/
├── map_manifest.yaml
├── trajectory.pcd
├── transformations.pcd
├── CornerMap/<keyframe>.pcd
├── SurfMap/<keyframe>.pcd
├── Scans/<keyframe>.pcd
└── SCDs/<descriptor>.scd
```

`map_manifest.yaml` 的 `schema_version` 用于阻止不兼容的新格式被旧程序静默加载；Scan Context 的 ring/sector 数也必须与运行程序一致。没有清单的历史地图仍可加载，但会被标记为 legacy map，并继续执行逐文件和维度检查。

地图写入必须使用独立的新目录；程序会拒绝包含旧产物的输出目录。建图期间目录中存在
`.lvi_sam_mapping_in_progress`，只有全部 PCD/SCD 和 manifest 成功提交后才会删除；定位模式
会拒绝仍带该标记的未完成地图。不要一边定位读取、一边覆盖同一目录中的地图文件。

## 3. 相机数据能否用于跨会话重定位

可以，但不建议无选择地保存整段原始视频。用于重定位的最小视觉地图应保存：

- 关键帧时间戳，以及与最近激光关键帧的关联；
- 建图时的相机/IMU 位姿；
- BRIEF 描述子、像素关键点和归一化关键点；
- 可参与 PnP 的三维地标或与激光深度关联的关键点；
- 相机模型、内参、畸变、相机—IMU—激光外参版本；
- DBoW2 词典和 BRIEF pattern 的版本标识；
- 可选的压缩灰度关键帧，仅用于重新提取特征、调试或人工审查。

建议目录：

```text
map_directory/VisualMap/
├── visual_manifest.yaml
├── keyframes.yaml
├── descriptors.bin
├── landmarks.bin
└── Images/                 # 可选，不是重定位必需项
```

跨会话视觉重定位必须采用以下链路：

```text
当前图像 → DBoW2 候选 → BRIEF 匹配 → PnP/RANSAC
        → 对应先验激光关键帧 → 当前激光与先验子地图 ICP
        → 创新量/适应度门控 → 接受重定位
```

当前代码的 DBoW2 数据库仍是进程内数据库，只能完成同一运行会话的视觉回环；尚未把视觉关键帧数据库作为正式地图文件加载。因此，新地图清单中的 `VisualMap/visual_keyframes.yaml` 是预留的可选扩展，不应把它误认为已经启用的视觉跨会话重定位。Scan Context + ICP 仍是当前已实现的跨会话重定位路径。

## 4. RTK/GPS 输入契约

`gpsTopic` 的消息类型是 `nav_msgs/Odometry`，不是原始 `NavSatFix`。上游必须先完成：

1. 经纬度到局部 ENU/地图坐标转换；
2. 天线杆臂补偿；
3. 正确填写 `header.frame_id`；
4. 把 RTK 解状态转换为可信的 XY/航向协方差；
5. 双天线航向不可用时，不得伪造低方差四元数。

建图时 `useGpsFactor` 为显式开关；定位时 `Loc.useRTKAssist` 与它相互独立。RTK 数据依次经过时间、有限值、协方差、坐标帧、绝对创新量和归一化创新量检查。建图 GPS 因子还使用 robust kernel，避免单次异常值强行拉动整张位姿图。

建图时建议把 `gpsExpectedFrame` 显式设为上游转换器实际发布的坐标帧；定位时用 `Loc.rtkExpectedFrame`。当前 Nav2 工程的 `rtk_to_liosam_odom.py` 默认发布 `odom`，所以充电场景配置使用 `odom`。这里的数据仍须与保存的 LIO 先验地图对齐；`frame_id` 名称一致不等于系统会自动完成坐标变换。留空只用于兼容尚未补齐 `frame_id` 的旧数据。

定位初始化默认要求连续多帧稳定 RTK。`Loc.rtkUseHeading=true` 时还必须提供有效的航向协方差。持续定位辅助采用协方差自适应权重，只修改扫描匹配初值，最终位姿仍由 scan-to-map 优化决定。

推荐原则：

- 普通建图和普通先验地图定位：默认关闭 RTK 因子/辅助；
- 需要全局地图对齐的首次建图：开启 `useGpsFactor`，使用低频、鲁棒约束；
- 充电站定位：RTK 可用于初始化和弱辅助，但 Scan Context/视觉/ICP 应保留为独立验证和降级路径；
- RTK fix 状态不可靠、协方差未正确填写或地图坐标未对齐时，必须关闭融合。

## 5. 兼容性原则

- 保留原 LVI-SAM 的 DBoW2/BRIEF 消息链路和 LIO-SAM 的主要话题命名，避免破坏已有 bag/launch。
- 新增参数均有安全默认值；全局 GPS 因子默认关闭。
- 旧地图无 manifest 时兼容加载；有 manifest 时执行严格版本和 Scan Context 维度检查。
- `mode`/`scene` 是入口 launch 的便捷选择，显式 `lidar_params_file` 始终具有最高优先级。
- MID360、相机和 IMU 外参必须来自同一次标定；示例配置不能作为实机稳定性结论。
