cmake_minimum_required(VERSION 3.5)
project(lvi_sam)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -pthread -fopenmp")

# ============ find dependencies ============
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(GTSAM 4.0.3 REQUIRED)   # 显式锁定 4.0.3，与 scripts/install_deps.sh、Dockerfile 一致，避免误装 4.2 引入 API 漂移
find_package(Eigen3 REQUIRED)        # 修复：原 LVI-SAM-ROS2 硬编码 /opt/eigen，改为标准 find_package
find_package(OpenMP REQUIRED)
find_package(Ceres REQUIRED)         # VIS(VINS) 需要
find_package(Boost REQUIRED COMPONENTS atomic filesystem program_options system timer)
find_package(livox_ros_driver2 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

set(dependencies
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  OpenCV
  livox_ros_driver2
)

# ============ 激光 LIS（来自 fork，已含 MID360/RTK/ScanContext/LoadPriorMap）============
add_executable(${PROJECT_NAME}_imuPreintegration src/lidar_odometry/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration gtsam)

add_executable(${PROJECT_NAME}_mapOptimization
  src/lidar_odometry/mapOptmization.cpp
  include/sc/Scancontext.cpp
)
target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam)

# ============ 视觉 VIS（来自 LVI-SAM-ROS2，ROS2 化 VINS-Mono）============
file(GLOB_RECURSE visual_feature_files
  "src/visual_odometry/visual_feature/*.cpp"
  "src/visual_odometry/visual_feature/camera_models/*.cc"
)
file(GLOB_RECURSE visual_estimator_files
  "src/visual_odometry/visual_estimator/*.cpp"
)
file(GLOB_RECURSE visual_loop_files
  "src/visual_odometry/visual_loop/*.cpp"
)

add_executable(visual_feature_node ${visual_feature_files})
target_link_libraries(visual_feature_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(visual_estimator_node ${visual_estimator_files})
target_link_libraries(visual_estimator_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(visual_loop_node ${visual_loop_files})
target_link_libraries(visual_loop_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

# ============ 依赖绑定 ============
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ${dependencies})
ament_target_dependencies(${PROJECT_NAME}_mapOptimization ${dependencies})
ament_target_dependencies(visual_feature_node ${dependencies})
ament_target_dependencies(visual_estimator_node ${dependencies})
ament_target_dependencies(visual_loop_node ${dependencies})

# ============ 安装 ============
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/gps_to_cartesian_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gps_to_cartesian_node
)

install(TARGETS
  ${PROJECT_NAME}_imuPreintegration
  ${PROJECT_NAME}_mapOptimization
  visual_feature_node
  visual_estimator_node
  visual_loop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
