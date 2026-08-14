# 改动汇总与稳定性审阅记录

本文档用于后期定位“为什么改、改了哪里、正常路径是否变化、如何验证”。当前审阅以已知可运行
分支提交 `bf3d92e` 为代码比较锚点，同时覆盖本增强版相对原 LIO/LVI-SAM 链路的功能变化。

版本锚点（用于复现和排查）：

- **状态接口基线**：`d87189f`，只包含观测型 `LocalizationStatus`，不包含复位订阅联动；
- **功能实现基线**：`4bb5ca5`，包含状态接口、`LocalizationReset`、IMU/VIS 代次保护和
  失败复位联动；
- `4bb5ca5` 之后的 `f1c13ac` 等提交只修订文档引用，不改变算法代码。后续若出现新的代码提交，
  应以该代码提交重新执行本文档第 6 节的 Orin 验收，而不能只看提交能否编译。

审阅日期：2026-08-14。远端功能分支为 `agent/visual-rviz-defaults`；部署时应显式检出该分支，
不要把本分支误认为 `main`。在目标机上用 `git rev-parse HEAD` 记录实际版本。

## 0. 结论速览

本轮改动可以分为“算法外围增强”和“接口/工程治理”两层。正常、有限、时间单调的输入下，
原有 LIS 点云匹配、IMU 预积分、GTSAM 图优化、VINS 滑窗以及 BRIEF+DBoW2 回环计算仍按原顺序
执行；较大的变化集中在模式选择、先验地图持久化/重定位、错误数据隔离、状态输出和跨节点复位。

| 关注点 | 当前结果 | 对原有算法的影响 |
|---|---|---|
| 建图/先验地图定位 | 通过 `mode` 和 `Loc.EnableFlag` 显式区分，地图以 manifest 事务方式保存和加载 | 仅定位模式加载先验地图；mapping 不会误读旧地图 |
| 回环 | 保留 LIS 的 Scan Context+ICP，并接入 VIS 的 BRIEF+DBoW2 候选交叉验证 | 不替换原回环求解器，新增候选必须经过 LIS 几何验证 |
| ROS 2/依赖 | 统一 launch、配置预检、`BUILD_VISUAL` 开关和内部图像适配层 | 纯 LIS 可独立构建；视觉不再强依赖 `cv_bridge` ABI |
| IMU/安装外参 | external/MID-360 profile、机器人安装 profile 和相机 profile 分离 | 传感器更换只改 YAML；算法不查询 TF 猜外参 |
| RTK | 时间、帧、协方差、创新量和质量门控；默认关闭 | 低质量 RTK 被拒绝，不直接污染图优化 |
| 状态接口 | 发布结构化 `LocalizationStatus`，保留旧字符串状态话题 | 只观察/门控，不反向改变匹配和图优化 |
| 复位联动 | 发布 `LocalizationReset`，按 `reset_id` 清理 IMU/VIS 跨代队列 | 地图基准改变时短暂等待新锚点，避免旧数据跨代拼接 |
| 可视化/文档 | RViz 默认项、部署脚本、配置和验收文档统一 | 只改变启动便利性和诊断手段 |

下列内容**明确没有被本轮重写**：LOAM 特征定义、scan-to-map 优化目标、因子图噪声模型、
VINS 滑窗因子、DBoW2/BRIEF 描述子和 Nav2 的上层规划逻辑。若这些模块的正常结果发生变化，
应优先检查传感器时间戳、外参、输入质量和配置版本，而不是把变化归因于状态接口本身。

## 1. 设计原则

1. 纯 LIS 建图路径必须能独立编译、启动和降级，不依赖相机、Ceres 或视觉节点。
2. 对合法、单调、有限的原始数据，保留原 LOAM、IMU 预积分、因子图和 VINS 优化主流程；
   新增逻辑主要位于输入校验、配置校验、跨线程同步和可选功能边界。
3. 先验地图、视觉、回环和 RTK 都是显式开关，不用隐式探测改变算法模式。
4. 错误数据采用“拒绝该样本并告警”，地图文件采用“事务未完成即不可定位”的策略。
5. 不引入 YAML 生成器或运行时继承层。配置保留为显式场景文件，通过校验器防止重复字段漂移。

## 2. 改动总表

| 模块 | 主要文件 | 改动 | 对正常路径的影响 |
|---|---|---|---|
| 构建与依赖 | `CMakeLists.txt`、`package.xml`、`scripts/*`、Docker | C++17；GTSAM 4.x；`BUILD_VISUAL`；标准 rosdep；复用已安装 Livox 驱动；构建前配置预检 | LIS 算法不变；视觉可完全从构建中移除 |
| OpenCV 兼容 | `image_conversion.hpp`、视觉图像回调 | 使用内部 `sensor_msgs/Image -> cv::Mat` 适配，不链接 `cv_bridge` | 像素语义不变；避免 ROS OpenCV 4.5/JetPack 4.8 同进程 ABI 冲突 |
| 统一启动 | `run.launch.py` | `mode × scene` 选配置；VIS/RViz 默认开启；话题、地图目录、TF 发布显式覆盖；启动前检查文件 | 默认会多启动 VIS/RViz；纯 LIS 用开关恢复最小链路 |
| 配置治理 | `config/*.yaml`、`validate_config.py`、`config/README.md` | 配置集中；同场景 mapping/localization 传感器及外参一致性；数值/资源/接口校验 | 不改变合法参数；错误配置在节点创建前失败 |
| Mid-360 前端 | `imageProjection.hpp`、`featureExtraction.hpp` | 容器 RAII、边界/有限值/时间检查、队列上限；保留最后一个 Livox 点；修正特征排序末端遗漏 | 合法帧计算链不变；修复原先最后点/末端元素遗漏 |
| IMU 预积分 | `imuPreintegration.cpp`、`internal_odom_metadata.hpp` | 有限值和单调时间检查；智能指针；队列上限；图优化重置编号；帧语义统一为物理 LiDAR | 正常积分方程不变；图优化发生跳变时主动重建积分状态 |
| 地图优化 | `mapOptmization.cpp`、`file_tools.hpp` | 线程快照、输入门限、Scan Context+ICP、地图清单/事务、定位状态机、RTK 门控 | 普通 mapping 主顺序仍为初值→局部图→降采样→scan-to-map→因子图→发布 |
| 先验地图定位 | `mapOptmization.cpp`、localization YAML | 加载 PCD/SCD/manifest；Scan Context 全局重定位；丢失检测和强制重定位服务 | 仅 `Loc.EnableFlag=true` 进入；mapping 不执行先验地图分支 |
| 地图持久化 | `mapOptmization.cpp`、地图格式文档 | 新/空目录要求；写入中标记；逐帧 PCD/SCD；最终 manifest 原子提交；帧和维度核对 | 改变旧版“可覆盖目录”习惯，避免混合旧地图；不改变在线里程计 |
| RTK/GNSS | `utility.hpp`、`mapOptmization.cpp`、charging 配置、GPS 辅助脚本 | 时间、帧、协方差、位置/航向、创新量门控；Huber；定位自适应融合；安全更新 YAML | 默认关闭；质量差输入不会进入因子图或定位修正 |
| VIS Feature | feature tracker 源码 | 图像编码/尺寸校验；频率 `0=逐帧`；深度缓存和重启清理；话题统一 | 合法图像仍走原光流/特征跟踪；错误帧不进入跟踪器 |
| VIS Estimator | estimator 源码 | 消息通道校验；IMU/特征/里程计队列和时间检查；重启原子清理；安全滑窗查找；线程退出 | 原 VINS 优化因子和滑窗算法保留；坏样本和跨序列样本被拒绝 |
| VIS Loop | loop detector 源码、DBoW2 词表读取 | 三路近似时间同步；队列上限；词表健壮读取；可配候选门限；RAII 关键帧；退出 join | BRIEF+DBoW2+PnP 逻辑保留；候选仍须由 LIS 点云 ICP 验证 |
| 公共接口 | `topic_names.hpp`、`package_assets.hpp`、`visual_frame_conventions.hpp` | 话题、包资源、坐标约定和里程计元数据集中定义 | 消除多处字符串/矩阵魔法值，不增加运行节点 |
| 本体点云过滤 | `utility.hpp`、六套 LIS profile | 过滤盒显式选择 `lidar`/`base` 坐标；通用实机配置启用原 LiDAR-frame 小盒；base 模式只使用配置外参 | 不读取 TF；更换 IMU不改变过滤语义，盒边界仍需实机调 |
| MID-360 视觉联调 | `params_camera_mid360.yaml`、`run.launch.py` | 按 `imu_source` 自动选相机 profile；将实机 `T_cam_radar` 转换为算法深度帧外参 | LiDAR 深度已启用；里程计先验和全局对齐分阶段启用 |
| RViz 与文档 | `rviz2.rviz`、README、`docs/*` | 默认显示注册点云、轨迹和回环诊断；补充部署、使用、架构、接口与验收文档 | 只影响可视化和操作流程 |

## 3. 原逻辑兼容性结论

### 3.0 定位状态接口与分阶段复位事件

本轮没有重写点云匹配、局部地图、因子图或 VINS 滑窗算法。首先增加了
`lvi_sam_msgs/msg/LocalizationStatus`，由 `lvi_sam_mapOptimization` 以
`/lio_sam/localization/status` 发布，供上层状态机按数值状态进行门控：
`MAPPING`、`RELOCALIZING`、`VERIFYING`、`TRACKING`、`DEGRADED`、`LOST`。
其中 `VERIFYING` 与 `DEGRADED` 是观测层质量状态，不会反向驱动旧算法；
`pose_valid` 也不等价于可运动，正常导航应只在 `TRACKING` 放行。

随后增加了独立的 `LocalizationReset` 事件接口，默认话题为
`/lio_sam/localization/reset`，并通过 `localization_reset_topic` 统一配置。
地图优化节点仍是 `reset_id` 的唯一所有者，旧的里程计协方差元数据继续发布，
因此旧消费者保持兼容。地图重定位、强制重定位、回环图修正会清理 IMU/VIS
跨代队列；VINS 或 IMU 自身故障只发布明确的下游复位请求，不修改 LiDAR 匹配
和地图因子图。所有接收者按 `(source,event_id)` 去重，避免多发布者同题自回环。

对应契约与测试见 `docs/LOCALIZATION_STATUS_INTERFACE.md`、
`docs/LOCALIZATION_RESET_INTERFACE.md` 以及 `test_localization_*_contract.cpp`。
这一层的实机验收仍需在 Orin 上按 mapping、定位、丢失和强制重定位四条路径
分别执行；在验收前，上层只能把 reset 事件作为诊断/协同信号，不能把它当作
Nav2 速度或 TF 指令。

严格按“先状态、后复位”验收时，可将 `d87189f` 作为状态接口专用基线（包含
`VERIFYING/DEGRADED`，不含复位事件订阅）；状态接口通过后，再切换到当前
`agent/visual-rviz-defaults` 功能分支（功能实现基线 `4bb5ca5`），逐项验证
`LocalizationReset`、IMU 队列清理、旧代数据丢弃和 VINS 重启联动。
中间提交 `6fe3e07`、`6333b29`、`8fe1329` 分别对应复位契约、地图事件和
IMU 传播接入；`1500818`、`0497e69` 是 force/lost 验收器增强，便于出现问题
时精确回退。

### 3.1 纯激光建图

主执行顺序未被重排：Mid-360 CustomMsg → 去畸变 → LOAM 特征 → 初值更新 → 局部地图 →
scan-to-map → 关键帧/因子图 → 位姿与点云发布。以下变化属于确定性修复：

- 原 CustomMsg 转换漏掉 `point_num` 的最后一个点，现按实际点数完整转换；
- 原特征排序区间遗漏末端元素，现使用完整闭区间；
- 原固定数组和手动资源改为等价 RAII 容器；
- 非有限、倒序或越界数据在进入 PCL/GTSAM 前丢弃；合法数据不经过额外估计环节；
- `mappingProcessInterval=0` 表示处理每个已接受雷达帧，避免 10 Hz 抖动导致隔帧。

### 3.2 IMU 与图优化衔接

预积分公式、噪声模型和 GTSAM 优化结构保留。新增重置编号只在 GPS、外部位姿或回环导致历史
图位姿整体修正后变化；消费者收到变化后放弃跨坐标跳变的旧积分，从下一次图优化修正重新初始化。
该行为会在图修正时短暂少发布一段预测，但比把修正前后的积分连续拼接更稳定。

`/odometry/imu` 的协方差前八个槽位用于兼容原 LVI-SAM 的内部初始化元数据，不是统计协方差。
Nav2 或通用融合器应使用 `/lio_sam/mapping/odometry`，或由集成层发布带真实协方差的独立话题。

### 3.3 视觉链路

原光流、VINS 滑窗、BRIEF、DBoW2 和 PnP 核心计算未替换。新增适配层只完成编码检查和灰度
转换；`rgb8` 输入按标准 RGB 权重转灰度。线程改动用于确保重启、断流、时间回拨和退出时不会
残留旧队列或后台线程。

VIS 默认启动是操作策略变化，不是 LIS 算法依赖。未完成相机/IMU/LiDAR 标定时，VIS 可以运行
单目惯性链路；取得实机 `T_cam_radar` 后只先启用 LiDAR 深度，LiDAR 里程计先验和在线
LiDAR-camera 全局对齐继续关闭，避免一次引入多条耦合链路。

2026-08-13 收到的标定约定为 `p_camera_optical = T_cam_radar · p_lidar`。工程没有直接把
光学帧矩阵当作欧拉角使用，而是先左乘
`R_depth_optical=[[0,0,1],[-1,0,0],[0,-1,0]]`，得到相机原点处 ROS 前左上深度帧，再按
`Rz·Ry·Rx` 提取 `lidar_to_cam_r*`。转换结果为平移
`[-0.0819725930, -0.0441752998, 0.1449693849] m`、RPY
`[-0.0216126479, 0.5440360865, -0.0165802054] rad`。相机—IMU 外参使用
`T_imu_cam = T_imu_lidar · inverse(T_cam_lidar)` 组合；原始旋转矩阵行列式为 1，配置预检会
继续检查转换后旋转的正交性。附件没有畸变系数，因此只更新 K，畸变暂沿用此前 CameraInfo。

### 3.4 本次复审确认并修正的问题

- 针对部分 Orin ROS 2 Humble 镜像没有 `rclcpp::Time::to_msg()` 的情况，新增公共
  `toBuiltinTime()` 纳秒级转换，复位事件和结构化定位状态统一使用该适配，不改变消息时间戳
  语义；定位复位契约测试同时显式包含 `LocalizationStatus` 头文件，避免依赖其他消息的传递包含。
- 恢复 `/lio_sam/mapping/cloud_registered_raw` 的实际发布。原实现只创建了 publisher，
  注册高分辨率点云的代码整段处于注释状态；同时原代码把它放在受 `useRviz` 和关键帧非空条件
  控制的 `publishFrames()` 中。现在它在每个已接受的处理帧完成当前位姿更新后独立发布，不再
  依赖 RViz，也不再因为首个关键帧尚未保存而丢失首帧。输出使用现有的
  `cloudInfo.cloud_deskewed`（已投影、去畸变并经过距离/本体过滤的高分辨率扫描），按
  `transformTobeMapped` 转到 `odometryFrame`，时间戳沿用该扫描的 header。它不是未经处理的
  Livox 原始 CustomMsg；未通过输入校验、定位尚未初始化或没有进入处理周期的扫描不会发布。
- 视觉回环匹配图仍使用 OpenCV 2/3 时代的 `CV_GRAY2RGB` 宏，在 Orin 的 OpenCV 4.8
  编译失败；已改为命名空间化的 `cv::COLOR_GRAY2RGB`。GCC 关于 C++17 `std::pair` 参数传递
  变化的输出只是 ABI note，不是构建错误。
- 已确认 PCL binary writer 会拒绝空点云，因此保留逐关键帧 0 字节空特征标记；配套定位加载器
  先检查文件大小并把它解释为空特征，不会把该标记交给 `loadPCDFile()`。这样既保持索引连续，
  又不为正常建图增加一个 PCL 异常分支。
- 去畸变模块读取 IMU 里程计重置编号时，已改用公共元数据索引，避免再次出现协方差魔法下标。
- 多套配置曾把源码读取的顶层 `useRviz` 错误缩进到 `Loc`，因此修改该值不会生效；现已统一
  移到顶层，并由预检器拒绝错误层级；mapping 文件中不会读取的定位参数也已移到对应
  localization 文件。launch 的 `enable_rviz` 与 LIS 数据发布开关已分别说明。
- 配置预检原先只比较六套 LIS 的公共话题/帧，现进一步核对每个场景的 mapping/localization
  雷达扫描参数和 IMU-LiDAR 外参，并检查旧 `params.yaml` 与通用定位配置一致。
- 新增 `params_imu_external.yaml` 与 `params_imu_mid360.yaml` 两套 IMU profile，以及
  `imu_source:=external|mid360` 启动选择。话题、噪声、重力参数、IMU-LiDAR 外参和加速度
  单位按物理 IMU 成组切换；MID-360 原始 `g` 单位在 LIS 与 VIS 入口统一转换为 `m/s²`。
  MID-360 的 IMU-LiDAR 旋转和平移参考 FAST-LIO 官方 `mid360.yaml`。开启视觉时自动选择
  `params_camera_mid360.yaml`；LiDAR-camera 使用实机标定，相机—IMU 杆臂仍需最终实测确认。
- 将原来含义重叠的 IMU/安装外参拆分为两层：IMU profile 使用明确方向的
  `imuToLidar*` 参数；新增 `params_mount_robot.yaml` 保存 `T_base_lidar`。MID-360 没有
  有效姿态消息时以安装标定初始化倾角，外置 IMU 继续使用姿态消息。安装参数也用于融合
  `odom→base_link` 输出，因此新配置不再读取 TF tree。本轮进一步移除缺少安装 profile 时的
  TF 查询回退，并取消 IMU/LiDAR 与 LiDAR-camera 外参的隐式单位矩阵/零平移默认值；启用相关
  链路却缺少 YAML 外参时立即失败。
- Livox `CustomMsg` 的点时间出现包级乱序时，不再丢弃整帧；转换层会对有限点按
  `offset_time` 稳定排序后继续去畸变。空帧、点数越界和全非有限点仍会被明确拒绝。

## 4. 有意改变的运行约束

| 变化 | 原因 | 操作影响 |
|---|---|---|
| 建图目录必须为新目录或空目录 | 防止新旧 PCD/SCD 混合 | 每次测试使用带编号的新目录 |
| 本版本新地图要求完整 manifest，拒绝写入中地图 | 防止异常退出地图进入生产；无清单历史地图仍兼容 | 新建图看到完整保存日志后再切定位 |
| 无效/倒序传感器样本被丢弃 | 防止 NaN 或负 `dt` 污染优化器 | 持续告警时先修时间源，不调大算法门限 |
| VIS 与 RViz 默认开启 | 方便完整链路和现场可视化 | SSH 使用 `enable_rviz:=false`；纯 LIS 加 `enable_visual:=false` |
| `BUILD_VISUAL=OFF` 不生成视觉程序 | 隔离 Ceres/OpenCV 问题 | 对应运行必须显式 `enable_visual:=false` |
| RTK 输入必须是 map 对齐 Odometry | 因子图不能直接解释经纬度 | 上游完成投影、质量和杆臂处理 |

## 5. 配置与接口索引

- 唯一配置说明：[`../src/lvi_sam/config/README.md`](../src/lvi_sam/config/README.md)
- 稳定接口契约：[`INTERFACES_AND_STABILITY.md`](INTERFACES_AND_STABILITY.md)
- 地图格式：[`ARCHITECTURE_AND_MAP_FORMAT.md`](ARCHITECTURE_AND_MAP_FORMAT.md)
- 远程测试流程：[`REMOTE_TEST_AND_CHANGES.md`](REMOTE_TEST_AND_CHANGES.md)
- Orin 部署：[`DEPLOY_ORIN.md`](DEPLOY_ORIN.md)

不要同时修改 YAML、launch 默认值和 C++ 默认值来表达同一个现场差异。部署话题/目录放 launch，
算法与标定放 YAML，C++ 默认值仅作为缺参保护。

## 6. 已完成检查与实机待验

已完成的离线检查：

- 六套活动 LIS、旧兼容 LIS 和 VIS YAML 解析及契约预检；
- Python launch/辅助脚本语法检查；
- XML、Markdown 本地链接、文本空白检查；
- 公共纯 C++ 头的最小 C++17 语法测试；
- 差异逐链路审阅，包括 mapping、localization、IMU、VIS、loop、RTK 和地图保存。

当前 Windows 审阅环境没有 ROS 2 Humble、PCL、GTSAM、Ceres 和 Livox 消息包，因此不能在本机
完成全量 C++ 链接或运行算法。合并前仍必须在 Orin 执行：

```bash
bash scripts/build.sh --clean
source install/setup.bash
colcon test --packages-select lvi_sam
colcon test-result --verbose
```

实机依次验证纯 LIS 静止/直行/转弯、完整建图保存、先验地图重定位、闭环、相机断流恢复、
低质量 RTK 拒绝以及进程退出。验收项和命令见 `INTERFACES_AND_STABILITY.md` 第 8 节。

## 7. 已知边界

- 视觉 DBoW2/BRIEF 数据库仍只存在于当前运行内存；跨会话重定位由 Scan Context + ICP 完成。
- 当前地图清单为 schema 1，只持久化 LiDAR 地图、关键帧位姿和 Scan Context；相机图像、描述子、
  标定版本与三维关联点尚未形成可加载的数据集。
- RTK 内核不会直接消费 `sensor_msgs/NavSatFix`，也不能代替接收机质量判断、坐标投影和杆臂补偿。
- VIS 只提供松耦合候选和内部里程计，不应单独发布 Nav2 的 `map -> odom -> base_link` 主 TF 链。

## 8. 变更时的回归规则

1. 修改同一场景的雷达型号、扫描参数或 IMU-LiDAR 外参时，同时更新 mapping/localization 文件。
2. 新参数必须在 YAML、`utility.hpp`/视觉参数读取和 `validate_config.py` 三处保持一致。
3. 新话题必须登记到接口文档；VIS 话题同时加入 `topic_names.hpp` 和测试。
4. 改地图格式必须升级 schema，并提供旧版本读取或明确迁移工具。
5. 改正常计算路径时必须记录输入、旧结果、新结果和差异原因；仅“能编译”不能作为算法回归依据。
6. 上车测试失败时先关闭 VIS、RTK、外部位姿和回环恢复最小 LIS，再逐项开启，避免同时排查多条链路。
