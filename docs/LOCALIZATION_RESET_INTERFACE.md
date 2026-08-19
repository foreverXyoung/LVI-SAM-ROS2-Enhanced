# 定位事件接口（观察性）

`/lio_sam/localization/reset` 是面向上层状态机和诊断工具的结构化事件输出。
它不再是 LVI-SAM 内部估计器的控制输入。

## 基本契约

| 项目 | 值 |
|---|---|
| 消息 | `lvi_sam_msgs/msg/LocalizationReset` |
| 默认话题 | `/lio_sam/localization/reset` |
| QoS | reliable、depth 10、volatile |
| `reset_id` 所有者 | `lvi_sam_mapOptimization` |

- `event_id` 只标识同一发布源产生的事件。
- `reset_id` 反映原有 LIO-SAM 内部地图/图优化代次，不由事件发布函数递增。
- `reason`、`reset_imu` 和 `restart_visual` 描述事件的性质和建议动作，发布消息本身不执行这些动作。
- LVI-SAM 的 IMU 预积分、融合传播、视觉特征、视觉估计器和视觉回环节点不订阅该话题来清理内部状态。

## 与原算法的边界

内部算法继续使用原有数据通道：

1. `mapOptimization` 仅在原有图修正路径中递增 `imuPreintegrationResetId`；
2. 该代次通过 `mapping/odometry_incremental` 的兼容元数据传递；
3. IMU 预积分按原有 `reset_id` 路径重新初始化；
4. 状态和事件发布不得清空 IMU 队列、重建 GTSAM 图、重置 bias 或重启视觉滑窗。

这样可以保证增加上层状态接口不会改变已经验证过的点云匹配、IMU 预积分、图优化和视觉里程计时序。

## 数值安全保护

数值安全检查不属于状态机控制。若一次激光校正之前没有形成有效 IMU 预积分区间，
IMU 节点跳过本次图更新，等待下一帧，而不会构造零时长 IMU/Bias 因子。
若 GTSAM 更新仍抛出异常，节点只重置自身的短时传播状态并发布诊断事件，不能让状态事件反向触发第二次复位。

## 上层使用规则

上层状态机可以订阅事件用于：

- 记录重定位、强制重定位、地图修正、VINS 或 IMU 异常；
- 决定导航任务是否暂停、降速或请求人工处理；
- 触发独立且经过验收的恢复服务。

上层不得把该话题直接回接为 LVI-SAM 内部队列清理命令。未来若需要主动恢复，必须使用独立服务或动作接口，并单独完成 bag、实机和乱序时序测试。

## 验收要点

1. 发布任意 `LocalizationReset` 消息时，`/lio_sam/mapping/odometry` 和 `/odometry/imu` 不应因为该消息被清空或重新初始化；
2. 初次先验地图重定位后不应出现由零时长 bias 因子引起的 `IndeterminantLinearSystemException`；
3. 原有图修正导致 `reset_id` 变化时，兼容路径仍能工作；
4. 启用或关闭视觉节点不改变上述边界。
