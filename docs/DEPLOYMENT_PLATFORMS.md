# 平台部署前期调研：AGX Orin 与 x86 笔记本

> 目的：在真正上板 / 上机之前，把两类目标平台的**依赖差异与潜在坑**梳理清楚，
> 避免部署时"在 x86 编得好好的，到 Orin 上编译或运行直接崩"或反之。
>
> 范围：LVI-SAM-ROS2-Enhanced（LIS + VIS 话题级松耦合）的实际落地依赖。
> 状态：**前期调研，未经真机验证**。结论基于 JetPack 6.x / ROS 2 Humble / Ubuntu 22.04 公开资料与源码核对，
> 标注 `⚠️待真机验证` 的条目需在目标硬件上实测后再定论。

---

## 1. 两个目标平台画像

| 维度 | AGX Orin（边缘部署） | x86 笔记本（开发 / 验证） |
|---|---|---|
| 架构 | aarch64 (ARM64) | x86_64 (amd64) |
| 典型系统 | JetPack 6.x → **L4T R36.x = Ubuntu 22.04** | 原生 Ubuntu 22.04 |
| ROS 2 | Humble（官方目标系统，6.x 起对齐） | Humble |
| 算力 | 2048-core Ampere GPU + 12-core ARM | 取决于独显 / 核显 |
| 内存 | 32GB / 64GB LPDDR5（共享 GPU） | 16/32/64 GB |
| 磁盘 | 自带 eMMC / NVMe（注意写寿命） | SSD |
| 预装加速库 | CUDA / cuDNN / TensorRT（JetPack 自带） | 无（按需装） |
| 功耗/散热 | `nvpmodel` 限频、无主动散热易降频 | 无此约束 |

**核心结论先给**：从 JetPack 6（2024 起）开始，Orin 的底层系统就是 Ubuntu 22.04，
与 ROS 2 Humble 官方目标系统**完全对齐**——这一代"装系统"的兼容性比 JetPack 5（Ubuntu 20.04）时代好很多。
**真正的两地差异集中在**：GTSAM/Ceres/PCL 的**源码编译代价（内存/时长）**、**OpenCV 版本冲突**、**容器基础镜像**、
**电源散热限频**、以及**磁盘写寿命**。

---

## 2. 逐依赖调研（含风险等级与应对）

### 2.1 ROS 2 Humble 安装源
- **x86**：官方 ROS apt 源，一步到位。
- **Orin (JetPack 6.x)**：官方建议直接用 ROS 官方源装 `ros-humble-desktop`，与 x86 操作一致。
  早期 JetPack 5 / Isaac ROS 的 `isaac.download.nvidia.com` 源已不再必须（且常见 404，因版本不匹配）。
  - 若后续要叠加 Isaac ROS 加速包，再单独加 Isaac APT 源；本工程当前不依赖 Isaac ROS，无需。
- **风险**：低。
- **⚠️ 注意**：如果 Orin 仍是 **JetPack 5.x（Ubuntu 20.04）**，则 ROS 2 Humble 不在官方支持矩阵，
  需降级到 ROS 2 Galactic 或自行源码编译 Humble——**部署前务必 `head -n1 /etc/nv_tegra_release` 确认 L4T 版本**。

### 2.2 GTSAM（源码编译）—— 两地差异最大的一处
工程 `CMakeLists.txt:32` 为 `find_package(GTSAM REQUIRED)`，**无版本约束**（见 §4 修订项）。
Ubuntu 22.04 无 GTSAM apt 包，须源码编译（脚本/ Docker 均从 `borglab/gtsam` 编译）。

| 风险点 | x86 笔记本 | AGX Orin |
|---|---|---|
| 内存峰值 | ≥16GB 通常够，不易 OOM | **易 OOM**：32GB 共享显存，并行 `make -j$(nproc)`（12 核）极易吃满；Orin Nano 8/16GB 必崩 |
| 编译时长 | 数分钟~十几分钟 | 明显更久（ARM 单核弱），全量编译 30min~1h+ |
| 指令集 | 已设 `GTSAM_BUILD_WITH_MARCH_NATIVE=OFF`（脚本/ Docker 都关了），x86 安全 | 同上，`=OFF` 对 ARM 也安全（避免 `-march=native` 在跨机分发时出问题）|

- **必做（Orin）**：**配置 swap**。Autoware/Jetson 社区通用做法：`fallocate -l 32G /swapfile` + `mkswap` + `swapon`，写入 fstab 永久生效。
- **建议**：用 `ccache` 加速重复编译；`make -j` 限制在物理核数（Orin 用 `-j$(nproc)` 但配合 swap 更稳）。
- **风险**：高（Orin 上 OOM/超时是头号部署杀手）。
- **应对**：见 `scripts/install_deps.sh` 已新增 swap 检测（内存 <16GB 且无 swap 时告警并给命令）；Orin 上手动建 32GB swap 后再编。

### 2.3 Livox-SDK2 + livox_ros_driver2
- **Livox-SDK2**：C SDK，须**预编译安装到 `/usr/local/lib`**（工程 `livox_ros_driver2/CMakeLists.txt:250/253` 依赖该 `.so`）。
  x86 / ARM 都要走这一步，源码编译在两个平台都顺利（依赖少）。
- **livox_ros_driver2**：已作 **git submodule**（`src/livox_ros_driver2`，pin 1.1.1），随工作区 `colcon build`，无平台差异。
- **风险**：低~中。中风险点在于 **`liblivox_lidar_sdk_shared.so` 必须先在系统层就位**，若忘了装 SDK，编译 `livox_ros_driver2` 会找不到头/库——脚本已用文件存在性检测跳过已装情况，但仍需**先装 SDK 再 colcon build**。
- **⚠️ 网络**：SDK 与 driver 从 GitHub clone，`--depth 1` 已减小体积；Orin 上若 GitHub 访问不稳定，建议预先镜像或离线拷贝。

### 2.4 OpenCV 版本冲突（**Orin 特有高危坑**）
- **x86**：apt 装 `ros-humble-desktop` 会拉取 Ubuntu 的 `libopencv-dev 4.5.4`，系统干净，无冲突。
- **Orin (JetPack 6)**：系统**已预装 NVIDIA 定制版 OpenCV（常 4.8 / 4.10，带 CUDA 支持）**。
  当你再 `apt install ros-humble-desktop` 或 `libopencv-dev` 时，可能出现：
  - 版本并存导致 `cv_bridge` 链接的 OpenCV 与实际运行的不一致 → 运行期 `cv::xxx` 符号未定义 / ABI 崩溃；
  - 已知案例（Autoware on Orin 指南）明确要求**卸载 JetPack 自带 OpenCV、降级到 4.5.4** 才能编译通过。
- **应对候选**：
  1. **保持系统 OpenCV 不动**，编译时通过 `OpenCV_DIR` 指向 JetPack 的 OpenCV 配置，让 `cv_bridge` 与你的节点都用同一份；
  2. 或按 Autoware 做法降版本（但会失去 CUDA 加速的 OpenCV，对 VIS 图像前端不友好）。
- **风险**：高（Orin 上最常见的"编得过跑不起来"元凶）。
- **⚠️待真机验证**：选方案 1 还是 2，取决于是否需要 OpenCV 的 CUDA 加速（VIS 光流/特征提取若能用 CUDA 版 OpenCV 更好）。**这是上板后第一个要拍板的事**。

### 2.5 CUDA / cuDNN / TensorRT
- **Orin**：JetPack 已预装，路径在 `/usr/local/cuda`。**容器**内若用 L4T 镜像（`nvcr.io/nvidia/l4t-*`）则自带；
  若误用 x86 的 `ros:humble` 镜像则无 CUDA，且无法在 ARM 上跑 x86 二进制。
- **x86**：本工程不需要 GPU 也能跑（SLAM 是 CPU 计算为主），仅可视化/可选加速用到。若要用 NVIDIA 容器运行时需另行配置。
- **风险**：中（仅 Orin 容器选错基础镜像时触发）。

### 2.6 PCL / Ceres / Eigen
- `CMakeLists.txt:30/35` 依赖 `PCL`、`Ceres`（VIS/VINS 需要）。
- **x86**：apt 直接装 `libpcl-dev` / `libceres-dev`，秒级。
- **Orin**：apt 同样可用（Ubuntu 22.04 仓库有），**无需源码编译**；但 Ceres 若源码编译会比 x86 慢。
  - 建议 Orin 上也用 apt 版，**不要**为 Ceres 走源码（除非需要特定版本）。
- **Eigen3**：脚本已修掉原 LVI-SAM-ROS2 硬编码 `/opt/eigen`（`CMakeLists.txt:33` 改标准 `find_package(Eigen3)`），x86 / ARM 均无碍。
- **风险**：低。

### 2.7 Docker 镜像（**平台不匹配会直接失败**）
- **当前 `Dockerfile` 问题**：`FROM ros:humble` 是 **x86 镜像**。在 Orin 上 `docker build` 会因架构不匹配或基础镜像不存在而失败。
- **正确做法**：
  - x86：`FROM ros:humble`（当前默认，正确）。
  - Orin：**必须**用 L4T 镜像，标签匹配设备 L4T 版本：
    - `nvcr.io/nvidia/l4t-ros2:humble-r36.4.0`（NGC，需登录 `nvcr.io`）
    - 或社区免登录：`dustynv/ros:humble-ros-base-l4t-r36.4`（标签须与 `head -n1 /etc/nv_tegra_release` 的 R36.x 对齐）
  - **L4T 版本不匹配**会导致容器内 `apt` 源或底层库不兼容——务必先查 L4T 再选标签。
- **修订**：`Dockerfile` 已把 `FROM` 改为参数化（`ARG BASE_IMAGE=ros:humble`），Orin 构建用：
  `docker build --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-ros2:humble-r36.4.0 -t lvi-sam-orin .`
- **风险**：高（不改则 Orin 容器路线完全走不通）。

### 2.8 电源 / 散热 / 限频（Orin 运行期特有）
- Orin 在无主动散热或高负载下会 **`nvpmodel` 降频**，导致 SLAM 实时性抖动（LIO-SAM 对单帧处理时延敏感）。
- **应对**：
  - 上电后 `nvpmodel -m 0`（最大性能模式）+ `jetson_clocks`（锁频）；
  - 保证散热（风扇 / 导热）；长时间运行监控 `tegrastats` 温度。
- x86 笔记本无此约束（但注意插电 vs 电池的性能差异）。
- **风险**：中（影响长时间运行稳定性，不影响编译）。

### 2.9 存储 / 磁盘写寿命
- Orin 常用 **eMMC**（写寿命有限）。本工程 `.gitignore` 已排除 `*.pcd *.scd *.log *.bag`，
  但运行时地图保存（`savePCDDirectory`）、日志仍会写盘。
- **应对**：地图/日志写到 NVMe 或挂载外部存储；避免 eMMC 高频写。
- **风险**：低~中。

---

## 3. 部署形态建议

| 阶段 | 推荐平台 | 说明 |
|---|---|---|
| 算法调通 / 参数 / 话题联调 | x86 笔记本（或台式） | 编译快、内存足、调试方便；用仿真 bag / 录像验证 VIS↔LIS 接线 |
| 最小验证闭环（M1+M2+部分 M3） | x86 为主，Orin 并行 | 先在 x86 把编译与话题接线跑通，再上 Orin 验证实时性 |
| 真实车/站场部署 | AGX Orin | 边缘算力 + 低功耗；需处理 §2.4 OpenCV 冲突 + §2.8 限频 |

**不建议**直接用 Orin 做首次全量编译验证（耗时 + OOM 风险高）。

---

## 4. 现有仓库脚本 / 构建的修订清单（本次已落地）

| # | 问题 | 风险 | 修订 | 位置 |
|---|---|---|---|---|
| 1 | Dockerfile 硬编码 `FROM ros:humble`（x86） | 高 | `FROM` 参数化为 `BASE_IMAGE`，Orin 用 L4T 镜像 + 加构建注释 | `Dockerfile` |
| 2 | `install_deps.sh` 无 swap 检测，Orin 编 GTSAM 易 OOM | 高 | 新增 swap 检测：内存 <16GB 且无 swap 时告警并给出 32GB 创建命令 | `scripts/install_deps.sh` |
| 3 | GTSAM 版本兼容 | 中 | 构建接受机器人现有的 4.0/4.1，脚本/Docker 默认安装 4.0.3；升级更高版本前应重新编译验证 | `src/lvi_sam/CMakeLists.txt` |
| 4 | OpenCV 冲突未在任何脚本处理 | 高（Orin） | 在 `docs/ENVIRONMENT.md` 与本文 §2.4 写明应对，运行时人工拍板 | 文档 |

> 第 3 项（CMake 版本约束）属于代码改动，影响编译行为；本次先写入文档与计划，
> 经你在 Orin 上确认 OpenCV 方案后再一并改，避免在未验证环境改出编译失败。

---

## 5. 待真机验证清单（⚠️）

- [ ] Orin 实际 L4T 版本（`head -n1 /etc/nv_tegra_release`），据此选容器标签。
- [ ] OpenCV 冲突：决定保持系统 CUDA 版还是降级 4.5.4（影响 VIS 性能）。
- [ ] GTSAM 在 Orin 上编译内存峰值 + 实际时长（确认 32GB swap 是否够）。
- [ ] `colcon build` 在 Orin 全量编译耗时与是否需 `ccache`。
- [ ] 实时性：满载时 LIO-SAM 单帧时延、`nvpmodel`/`jetson_clocks` 后的稳定性。
- [ ] livox_ros_driver2 + Livox-SDK2 在 Orin 的 MID360 实际出数（`/livox/lidar` + 标准 `sensor_msgs/Imu`，并用 `imu_topic` 统一接入）。
- [ ] VIS 三节点在 Orin 的 CPU 占用与温度（长时间运行 `tegrastats`）。

---

## 6. 快速核查命令

```bash
# 平台 / 版本核验
head -n1 /etc/nv_tegra_release        # Orin: 看 L4T R36.x
lsb_release -a                        # Ubuntu 22.04 核验
ls /opt/ros/humble >/dev/null && echo "ROS2 Humble OK"

# 依赖就位核验
ls /usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake && echo "GTSAM OK"
ls /usr/local/lib/liblivox_lidar_sdk_shared.so && echo "Livox-SDK2 OK"
pkg-config --modversion opencv4       # Orin 看系统 OpenCV 版本

# swap（Orin 必查）
free -h                               # 看 Swap 行；若 0 需建

# Orin 性能模式
sudo nvpmodel -m 0 && sudo jetson_clocks
tegrastats                            # 监控温度/利用率
```
