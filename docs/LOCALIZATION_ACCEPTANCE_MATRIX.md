# 定位状态接口验收矩阵

本阶段只验收观察性状态和事件输出。状态接口不得改变 LiDAR 匹配、图优化、IMU 预积分、TF 或视觉滑窗行为。

## 状态输出

| 场景 | 预期状态 | 算法侧禁止行为 |
|---|---|---|
| mapping | `MODE_MAPPING + MAPPING` | 不加载先验地图，不因状态发布重置估计器 |
| localization 初始化 | `RELOCALIZING → VERIFYING/TRACKING` | 不清空 IMU/VIS 队列，不改变 bias |
| 稳定跟踪 | `TRACKING` | 状态心跳不改变匹配阈值或因子图 |
| 质量下降 | `DEGRADED` | 不自动修改算法参数 |
| 定位丢失 | 至少一次 `LOST`，随后 `RELOCALIZING` | 事件发布不直接重建 IMU/VINS |
| 强制重定位服务 | 服务成功并返回 `RELOCALIZING` | 服务自身改变定位模式，但状态发布函数不得附带重置副作用 |

上层只在 `mode == MODE_LOCALIZATION && state == TRACKING` 时放行正常运动；其他状态采取暂停或降级策略。

## 事件输出

```bash
ros2 topic echo /lio_sam/localization/reset
```

可以观察到重定位、强制重定位、地图修正、VINS/IMU 故障等事件。验收时必须同时确认：

1. LVI-SAM 内部没有该话题的订阅者；
2. 单独发布测试事件不会中断 `/lio_sam/mapping/odometry`；
3. 单独发布测试事件不会清空 `/odometry/imu` 或视觉队列；
4. 原有 mapping odometry `reset_id` 在真正的图修正时仍按原逻辑工作；
5. 初次重定位后没有零时长 IMU/Bias 因子和 `b0` 奇异崩溃。

检查内部订阅者：

```bash
ros2 topic info /lio_sam/localization/reset --verbose
```

正常情况下可以有 map、IMU 或 VINS 的事件发布者，但 LVI-SAM 算法节点不应作为该话题的订阅者出现。

## Orin 构建与验证

```bash
cd /data/return_station_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select lvi_sam_msgs lvi_sam \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_VISUAL=ON

source install/setup.bash
ros2 run lvi_sam lvi_sam_validate_config
```

随后使用同一 bag 或同一组实机传感器输入，对照观察定位状态、里程计频率、进程存活和重定位后的恢复过程。
