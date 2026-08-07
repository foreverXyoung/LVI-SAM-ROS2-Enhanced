# LVI-SAM-ROS2-Enhanced — 可复现构建镜像
#
# 基础：ros:humble（Ubuntu 22.04 + ROS 2 Humble）
# 流程：apt/ROS 依赖 → GTSAM(源码) → Livox-SDK2(源码) → rosdep → colcon build
#
# 用法：
#   宿主机先初始化子模块（构建上下文需要它）：
#     git submodule update --init --recursive
#   docker build -t lvi-sam-ros2-enhanced .
#   docker run -it --rm --net=host --privileged -v /dev:/dev lvi-sam-ros2-enhanced bash
#
FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble
ARG GTSAM_VER=4.0.3

WORKDIR /ws

# ---------- 1) 系统 / 编译工具 ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl ca-certificates \
    libpcl-dev libopencv-dev libeigen3-dev libboost-all-dev libceres-dev \
    python3-pip python3-rosdep python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions ros-${ROS_DISTRO}-tf2* \
    ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-xacro \
 && rm -rf /var/lib/apt/lists/*

# ---------- 2) GTSAM（源码编译，Ubuntu 无 apt 包） ----------
RUN git clone --depth 1 --branch ${GTSAM_VER} https://github.com/borglab/gtsam.git /tmp/gtsam \
 && cd /tmp/gtsam && mkdir -p build && cd build \
 && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
 && make -j"$(nproc)" && make install && ldconfig \
 && rm -rf /tmp/gtsam

# ---------- 3) Livox Lidar SDK（源码编译，预装 /usr/local） ----------
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /tmp/livox_sdk \
 && cd /tmp/livox_sdk && mkdir -p build && cd build \
 && cmake .. -DCMAKE_BUILD_TYPE=Release \
 && make -j"$(nproc)" && make install && ldconfig \
 && rm -rf /tmp/livox_sdk

# ---------- 4) 复制工作区（含已初始化的 livox_ros_driver2 子模块） ----------
COPY . /ws

# ---------- 5) rosdep（跳过 gtsam） + 编译 ----------
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && rosdep update 2>/dev/null || true \
 && rosdep install --from-paths src --ignore-src -y --skip-keys gtsam \
 && colcon build --symlink-install --packages-up-to lvi_sam \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# 默认进入交互 shell（source 已持久化到 bashrc 便于使用）
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo "source /ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
