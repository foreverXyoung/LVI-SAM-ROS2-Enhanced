# LVI-SAM 定位状态与复位联动验收矩阵

本文档把“状态接口验收”和“复位联动验收”拆成两个阶段。第一阶段不依赖
`LocalizationReset`，用于证明状态输出不会改变原有 LiDAR 匹配、图优化、TF
或里程计行为；第二阶段才验证 IMU、视觉缓存和代次保护。

## A. 状态接口阶段

基线提交：`d87189f`。在该提交上只验收
`lvi_sam_msgs/msg/LocalizationStatus` 和 `/lio_sam/localization/status`。

| 场景 | 操作 | 必须观察到 | 失败判据 |
|---|---|---|---|
| mapping | `mode:=mapping`，使用全新输出目录 | `MODE_MAPPING + MAPPING`，`map_ready=false` | 结构化状态缺失，或原有 LIS 输出停止 |
| localization 初始化 | `mode:=localization`，加载完整先验地图 | `RELOCALIZING`，随后 `VERIFYING`/`TRACKING` | 只发布字符串状态，或状态跳过地图加载错误 |
| 稳定跟踪 | 保持传感器连续运动 | `MODE_LOCALIZATION + TRACKING` | 仅 `pose_valid=true` 就放行上层运动 |
| 质量下降 | 在可控低质量数据/遮挡段测试 | `DEGRADED`，不改变原有匹配决策 | 自动修改匹配参数或直接重置算法 |
| lost | 重现已知 bad-match 阈值 | 至少一个 heartbeat 的 `LOST`，随后 `RELOCALIZING` | `LOST` 永远不出现，或丢失后仍持续输出有效定位 |
| force | 调用已有 Trigger 服务 | 服务成功，状态回到 `RELOCALIZING` | mapping 模式被误接受，或状态不改变 |

第一阶段上层控制规则：只有
`mode == MODE_LOCALIZATION && state == TRACKING` 才允许正常运动；
`VERIFYING`、`DEGRADED`、`LOST` 和 `RELOCALIZING` 应保持/降速。

## B. 复位联动阶段

当前功能分支基线：`4bb5ca5`（远端 `agent/visual-rviz-defaults`）。

### B.1 地图代次复位

```bash
ros2 topic echo /lio_sam/localization/reset
```

确认以下事件由 `source=map_optimization` 发布：

- 接受先验地图初始位姿：`REASON_RELOCALIZATION`；
- 强制重定位：`REASON_FORCE_RELOCALIZATION`；
- 回环/图修正：`REASON_MAP_CORRECTION`；
- 丢失后重定位：`detail=localization_lost`。

每次地图基准变化只增加一次 `reset_id`。旧代次的
`/lio_sam/mapping/odometry_incremental`、`/odometry/imu_incremental` 和
视觉先验不应重新进入下游队列；收到低于当前代次的地图复位事件也必须被
丢弃，不能回退 IMU 图优化状态。

### B.2 视觉失败复位

在可控视觉失败测试中确认：

1. `source=visual_estimator`、`reason=VINS_FAILURE` 事件出现；
2. `restart_visual=true`，但 `reset_imu=false`；
3. LiDAR 地图和地图代次不变；
4. VINS 的特征/IMU/里程计队列被清空后重新初始化；
5. `odom -> vins_world` 对齐锚点重新建立，不沿用旧 TF 锚点。

### B.3 IMU 失败复位

确认 `source=imu_preintegration` 的失败事件不会在 IMU 节点自身重复清理刚到达
的新样本，同时 `TransformFusion` 会清空传播队列并等待新代次数据。

## C. Orin 命令模板

```bash
cd /data/return_station_ws/src/LVI-SAM-ROS2-Enhanced
git fetch origin agent/visual-rviz-defaults
if git show-ref --verify --quiet refs/heads/agent/visual-rviz-defaults; then
  git switch agent/visual-rviz-defaults
else
  git switch --track -c agent/visual-rviz-defaults origin/agent/visual-rviz-defaults
fi
git pull --ff-only origin agent/visual-rviz-defaults

cd /data/return_station_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to lvi_sam \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_VISUAL=OFF
source install/setup.bash

ros2 run lvi_sam verify_localization_status --case mapping --timeout 30
ros2 run lvi_sam verify_localization_status --case localization --timeout 60
ros2 run lvi_sam verify_localization_status --case force_relocalize --timeout 30
```

`lost`、`verifying` 和 `degraded` 必须在对应数据条件实际发生时运行；验收脚本
不会伪造传感器故障。视觉链路必须在 LIS 状态阶段通过后，再用
`-DBUILD_VISUAL=ON` 和已标定的 camera profile 单独验收。

## D. 当前证据边界

本地工程已通过配置校验、Python 语法检查、接口常量/状态映射单元测试和静态锁顺序
审阅；ROS 2、GTSAM、PCL、Ceres 的完整编译以及真实传感器四路径运行，必须以 Orin
上的命令输出作为最终证据。
