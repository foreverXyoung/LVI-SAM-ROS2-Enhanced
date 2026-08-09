# AGX Orin 部署手册（一步步）

> 配套调研： [`docs/DEPLOYMENT_PLATFORMS.md`](DEPLOYMENT_PLATFORMS.md)（Orin vs x86 依赖差异、风险矩阵）。
> 本手册给出在 **AGX Orin（JetPack 6.x → L4T R36.x = Ubuntu 22.04 + ROS 2 Humble）** 上
> 把 `LVI-SAM-ROS2-Enhanced` 部署并跑通 **真实 MID360 + 相机** 的逐命令步骤，
> 并在每一步嵌入"验证通过标志"，方便边做边确认。
>
> **一句话流程**：`setup_orin.sh --apply` → `install_deps.sh` → `build.sh` → 起激光驱动 → `run.sh` → 话题接线验证。
>
> ⚠️ 本手册基于源码核对与公开资料，**尚未在真机跑通**（无 Orin 真机）。凡标注 `⚠️待真机验证` 的，请在板上实测后回填结论。

---

## 0. 前置：把代码弄到 Orin 上

```bash
# 方式 A：git clone（推荐，含子模块）
git clone --recursive <your-repo-url> LVI-SAM-ROS2-Enhanced
cd LVI-SAM-ROS2-Enhanced

# 方式 B：U 盘 / scp 拷贝后，务必补子模块
# git submodule update --init --recursive   # 若拷贝来的不是 --recursive
git submodule status        # livox_ros_driver2 前应为空格（锁定 1.1.1），无 '+'/'-'
```

> 子模块 `livox_ros_driver2` 是 **git submodule**，漏了会编译报 `custom_msg` 找不到。

---

## 1. 系统 / 版本确认（不做后面全白搭）

```bash
bash scripts/setup_orin.sh          # 仅自检，打印 L4T / ROS / 内存 / OpenCV
```

**通过标志**：
- 看到 `L4T: # R36.x.x` → JetPack 6.x / Ubuntu 22.04，与 Humble 对齐 ✅
- `ROS 2 Humble 已安装` ✅
- 若提示 `L4T: R35.x`（JetPack 5 / Ubuntu 20.04）→ **Humble 不在官方支持矩阵**，需先升级 JetPack 到 6.x，或降级到 Galactic；**不要继续**。

> ⚠️待真机验证：确认你的 Orin 实际 JetPack 版本（JetPack 6.x 还是 5.x）。
> 当前 Docker / APT 路线默认按 6.x 走。

---

## 2. 性能模式 + swap（运行/编译都稳的前提）

```bash
bash scripts/setup_orin.sh --apply   # 自检 + 建 swap（如缺）+ 设最大性能模式
```

**通过标志**：
- 内存 <16GB 时自动建 32GB swap：`free -h` 看到 Swap 32G ✅
- `nvpmodel -m 0` + `jetson_clocks` 生效（可用 `sudo tegrastats` 看频率锁住）✅

> 为什么：`nvpmodel` 限频会让 LIO-SAM 单帧处理抖动；内存小 + 无 swap 编 GTSAM 必 OOM。
> 运行 SLAM 前建议保持性能模式；电池/散热受限时自行权衡。

---

## 3. OpenCV 单一依赖策略

JetPack 系统已自带 **CUDA 版 OpenCV（常 4.8 / 4.10）**；装 `ros-humble-desktop` 又会带 **4.5.4**。
旧版 VIS 直接链接预编译 `cv_bridge`，如果项目同时选择另一套 OpenCV，同一进程会加载两套
ABI，可能出现链接警告、`cv::xxx` 符号错误或运行期崩溃。

当前版本已使用工程内图像适配层替代 VIS 对 `cv_bridge` 的直接依赖。相机输入仍是标准
`sensor_msgs/msg/Image`，但 LVI-SAM 所有目标只链接一次 `find_package(OpenCV)` 的结果。
其他 ROS 进程继续使用系统 `cv_bridge` 不受影响。

**默认策略（推荐，零额外操作）**：让 CMake 使用目标机器默认可见的 OpenCV。当前实机的
`/usr/lib/cmake/opencv4/OpenCVConfig.cmake` 对应 4.8；标准 Ubuntu 22.04/ROS Humble 环境
通常会使用发行版 OpenCV 4.5。代码只使用兼容的 CPU OpenCV 4.x API。

**仅激光首测**：不构建 VIS，缩短编译和排障链路：
```bash
bash scripts/build.sh --lidar-only --clean
```

**多套 OpenCV 时显式选择**：仅在默认配置不符合预期时执行：
```bash
OpenCV_DIR=/usr/lib/cmake/opencv4 bash scripts/build.sh --clean
```

**通过标志**（编译后 / 运行前）：
```bash
# 不应输出 libcv_bridge；OpenCV 库应全部属于同一个 ABI 系列（例如全部为 .408）
for node in visual_feature_node visual_estimator_node visual_loop_node; do
  echo "===== $node ====="
  ldd "install/lvi_sam/lib/lvi_sam/$node" | grep -E 'cv_bridge|opencv' | sort -u
done
```
若仍看到 `libcv_bridge`，说明运行的是旧构建产物；使用 `--cmake-clean-cache` 或
`bash scripts/build.sh --clean` 后重新构建。若同一节点仍出现两套 OpenCV，则检查其余第三方库。

---

## 4. 安装依赖（ROS / GTSAM / Livox-SDK2）

```bash
bash scripts/install_deps.sh
```

脚本会自动：
- 装 apt / ROS 依赖；
- 源码编译 **GTSAM 4.0.3**（已锁版本；Orin 上并行度自动限 4 + 建议 swap，防 OOM）；
- 源码编译 **Livox-SDK2** 到 `/usr/local/lib`；
- 装 Python 依赖 + `rosdep`（跳过 gtsam）；
- Orin 上打印项目将使用的 OpenCV 配置，并说明内部图像适配层状态。

**通过标志**：
```bash
ls /usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake && echo "GTSAM OK"
ls /usr/local/lib/liblivox_lidar_sdk_shared.so && echo "Livox-SDK2 OK"
ldconfig -p | grep livox_lidar_sdk            # 有 .so
```
> 耗时预期：x86 数分钟~十几分钟；**Orin 上 GTSAM 全量 30min~1h+**（已限并行 + ccache 加速）。
> 若中途 OOM，扩大 swap 到 32GB 后重跑（脚本可重复执行，已装自动跳过）。

---

## 5. 编译工作区

```bash
bash scripts/build.sh
# 重编：bash scripts/build.sh --clean
```

`build.sh` 在 Orin 上已自动：OpenCV 匹配（`OpenCV_DIR`）、ccache 启动器、并行度限 4。

**通过标志**：
```bash
source install/setup.bash
ros2 pkg list | grep -E "lvi_sam|livox_ros_driver2"   # 两者都列出
# 5 个可执行应存在：
ls install/lvi_sam/lib/lvi_sam/
#   lvi_sam_imuPreintegration  lvi_sam_mapOptimization
#   visual_feature_node  visual_estimator_node  visual_loop_node
```

> 若 `Could NOT find GTSAM` → 重跑 §4 并 `sudo ldconfig`。
> 若 `liblivox_lidar_sdk_shared.so: cannot open` → Livox-SDK2 未装；重跑 §4。

---

## 6. 启动激光驱动（livox_ros_driver2）—— 单独终端

`run.sh` **只拉 lvi_sam 的 5 个节点，不含激光驱动**，需另起终端先起 MID360 驱动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# 用工程内或你已有的 MID360 launch（需指定 MID360 的 user_config 与 IP）
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**通过标志**（另开终端）：
```bash
ros2 topic list | grep -E "/livox/lidar|/IMU_data|/livox/imu"  # 点云和一个标准 IMU 话题存在
ros2 topic hz /livox/lidar                              # 有频率（如 ~10Hz）
```
> 驱动配置（MID360 的 `user_config.json` 里的 IP / 波特率）沿用你之前跑通定位模块的那套，本工程不重复提供。

---

## 7. 启动 LVI-SAM（LIS + VIS）

```bash
bash scripts/run.sh
# 真机定位用：bash scripts/run.sh lidar_params_file:=$(pwd)/src/lvi_sam/config/params_localization.yaml
# 仿真：      bash scripts/run.sh use_sim_time:=true lidar_params_file:=.../params_gazebo_localization.yaml
```

**通过标志**：launch 拉起 5 个节点无报错；RViz 能看到点云 / 轨迹。

---

## 8. 话题接线验证（核心：VIS↔LIS 松耦合是否真通）

按 `run.launch.py` 的接线设计，逐条核对：

| # | 接线 | 发布方 | 订阅方 | 验证命令 |
|---|------|--------|--------|----------|
| ① | `odometry/imu`（LIS→VIS 位姿/尺度先验） | `lvi_sam_imuPreintegration` | `visual_estimator_node` | `ros2 topic hz /odometry/imu` 有数据 |
| ② | `/lio_sam/deskew/cloud_deskewed`（LIS→VIS 激光深度） | `lvi_sam_mapOptimization` | `visual_feature_node` | `ros2 topic hz /lio_sam/deskew/cloud_deskewed` 有数据 |
| ③b | `/lvi_sam/vins/loop/match_frame` → remap → `lio_loop/loop_closure_detection`（VIS→LIS 回环候选） | `visual_loop_node` | `lvi_sam_mapOptimization` | 触发回环后看 mapOpt 日志 `performLoopClosure` |

```bash
# 一键拓扑核对
ros2 topic list | grep -E "odometry/imu|cloud_deskewed|match_frame"
ros2 node list   # 应见 5 个 lvi_sam_* 节点 + 驱动节点
```

> 注意 ③b 的 remap 方向：`visual_loop_node` 发 `/lvi_sam/vins/loop/match_frame`，
> `mapOptimization` 经 launch 的 `remappings=[('lio_loop/loop_closure_detection','/lvi_sam/vins/loop/match_frame')]`
> 收到。若回环没生效，先看 `visual_loop_node` 是否真的发了 match_frame（需 VIS 跑出回环）。

---

## 9. 实时性与散热（长时间运行）

```bash
sudo tegrastats --interval 2000   # 看温度（CPU/GPU）、频率、内存
```
- LIO-SAM 单帧时延敏感；保持 `nvpmodel -m 0` + `jetson_clocks`。
- 温度持续 >85°C 会降频 → 加散热 / 风道。
- 地图/日志写到 **NVMe 或外存**，避免 eMMC 高频写（见 `run.sh` 的 `pcd_directory`，默认 `/tmp/lvi_sam_maps`）。

> ⚠️待真机验证：满载时 LIO-SAM 单帧时延、`tegrastats` 温度曲线、VIS 三节点 CPU 占用。

---

## 10. 常见问题速查

| 现象 | 原因 | 解决 |
|------|------|------|
| 编译 GTSAM OOM / 卡死 | Orin 内存小 + 并行度满 | `bash scripts/setup_orin.sh --apply` 建 swap；build 已限并行 4 |
| 运行期 `cv::` 符号错误 / ABI 崩溃 | 旧构建缓存或其他第三方库仍引入第二套 OpenCV | 按 §3 清理重编并用 `ldd` 检查三个 VIS 节点 |
| `Could NOT find GTSAM` | 未装 / 未 ldconfig | 重跑 `install_deps.sh`，`sudo ldconfig` |
| `liblivox_lidar_sdk_shared.so: cannot open` | Livox-SDK2 未装 | 重跑 `install_deps.sh` 第 3 步 |
| `custom_msg` 找不到 | 子模块未初始化 | `git submodule update --init --recursive` |
| `visual_loop_node` 不发包 | VIS 未跑出回环（需足够位移/特征） | 正常：小场景难触发；不影响 LIS 主定位 |
| `/livox/lidar` 无数据 | 驱动未起 / MID360 IP 错 | 检查 §6 驱动 launch 与 user_config |
| 地图路径指向 `/home/lighter/...` | 旧版 launch 扁平覆盖失效 | 已修复为嵌套 `{'Loc':{'loadPCDDirectory':...}}`；重跑 `build.sh` |

---

## 11. （可选）Docker 路线

若想容器化部署（环境隔离、便于复现）：

```bash
git submodule update --init --recursive
# 标签须与 §1 查到的 L4T R36.x 一致
docker build --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-ros2:humble-r36.4.0 -t lvi-sam-orin .
docker run -it --rm --net=host --privileged -v /dev:/dev lvi-sam-orin bash
```
> ⚠️ 切勿用 x86 的 `ros:humble` 镜像在 Orin 上构建（架构不匹配）。
> 免登录社区镜像：`dustynv/ros:humble-ros-base-l4t-r36.4`（标签与 L4T 对齐）。
> Docker 内也应按 §3 用 `ldd` 确认每个 VIS 节点只加载一套 OpenCV。

---

## 12. 部署完成检查清单

- [ ] `setup_orin.sh --apply` 通过（L4T 6.x / ROS Humble / swap / 性能模式）
- [ ] `install_deps.sh` 通过（GTSAM 4.0.3 / Livox-SDK2 / rosdep）
- [ ] `build.sh` 通过（5 节点编译成功）
- [ ] MID360 驱动起，`/livox/lidar` + 标准 IMU 有数据；非 `/IMU_data` 时通过 `imu_topic` 覆盖
- [ ] `run.sh` 起 5 节点无报错
- [ ] 话题接线 ①②③b 全部 `ros2 topic hz` 有数据
- [ ] 三个 VIS 节点均不链接 `cv_bridge`，且各自只加载一套 OpenCV
- [ ] 长时间运行 `tegrastats` 温度/频率正常

---

## 附：本次为 Orin 部署新增 / 修订的文件

| 文件 | 变更 |
|------|------|
| `scripts/setup_orin.sh`（新增） | Orin 前置自检 + `--apply` 建 swap / 性能模式 / OpenCV 自检 |
| `scripts/install_deps.sh` | 加 ccache；GTSAM Orin 并行限 4；aarch64 检测与 OpenCV 自检 |
| `scripts/build.sh` | 支持显式 `OpenCV_DIR` + ccache + Orin 并行限 4；旧 `KEEP_SYSTEM` 仅提示弃用 |
| `src/lvi_sam/launch/run.launch.py` | 修复 `Loc.loadPCDDirectory` 嵌套覆盖（旧扁平写法无效，导致 Orin 地图路径指向 `/home/lighter/...`） |
| `src/lvi_sam/CMakeLists.txt` | 接受 GTSAM 4.0/4.1；移除 VIS 的 `cv_bridge` 链接并加入图像转换单元测试。 |
| `src/lvi_sam/include/lvi_sam/image_conversion.hpp` | 安全、可迁移的 ROS Image/OpenCV 内部适配层。 |
| `docs/DEPLOY_ORIN.md`（本文件） | 一步步 Orin 部署手册 |
