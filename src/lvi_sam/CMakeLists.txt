cmake_minimum_required(VERSION 3.5)
project(lvi_sam)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -pthread -fopenmp")

option(BUILD_VISUAL "Build the camera/VINS/DBoW2 pipeline" ON)
option(LVI_SAM_PREFER_ROS_OPENCV
  "Prefer the distro OpenCV ABI used by the prebuilt ROS cv_bridge" ON)

# ============ find dependencies ============
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
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
# Jetson often has a /usr/local CUDA OpenCV in addition to the Ubuntu/ROS
# build. A prebuilt cv_bridge remains linked to the distro ABI, so selecting
# /usr/local can put two incompatible OpenCV versions in one executable.
if(LVI_SAM_PREFER_ROS_OPENCV AND NOT OpenCV_DIR AND CMAKE_LIBRARY_ARCHITECTURE)
  set(_ros_opencv_dir "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/opencv4")
  if(EXISTS "${_ros_opencv_dir}/OpenCVConfig.cmake")
    set(OpenCV_DIR "${_ros_opencv_dir}" CACHE PATH
      "OpenCV config compatible with the ROS distro" FORCE)
    message(STATUS "LVI-SAM: preferring ROS-compatible OpenCV at ${OpenCV_DIR}")
  endif()
endif()
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
# LIO/LVI-SAM uses APIs available throughout the GTSAM 4.x line.  Keep 4.0.3
# as the reproducible installer default, but accept an existing 4.1.x system
# package used by deployed robot workspaces.
find_package(GTSAM 4.0 REQUIRED)
find_package(Eigen3 REQUIRED)        # 修复：原 LVI-SAM-ROS2 硬编码 /opt/eigen，改为标准 find_package
find_package(OpenMP REQUIRED)
if(BUILD_VISUAL)
  find_package(cv_bridge REQUIRED)
  find_package(Ceres REQUIRED)       # VIS(VINS) 需要
  find_package(Boost REQUIRED COMPONENTS atomic filesystem program_options system timer)
endif()
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

set(common_dependencies
  rclcpp
  ament_index_cpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_ros
  livox_ros_driver2
)

set(visual_dependencies
  ${common_dependencies}
  cv_bridge
)

# ============ 激光 LIS（来自 fork，已含 MID360/RTK/ScanContext/LoadPriorMap）============
add_executable(${PROJECT_NAME}_imuPreintegration src/lidar_odometry/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration gtsam)

add_executable(${PROJECT_NAME}_mapOptimization
  src/lidar_odometry/mapOptmization.cpp
  include/sc/Scancontext.cpp
)
target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam ${OpenCV_LIBS})

# ============ 视觉 VIS（来自 LVI-SAM-ROS2，ROS2 化 VINS-Mono）============
if(BUILD_VISUAL)
  file(GLOB_RECURSE camera_model_files
    "src/visual_odometry/visual_feature/camera_models/*.cc"
  )
  file(GLOB_RECURSE visual_feature_files
    "src/visual_odometry/visual_feature/*.cpp"
  )
  file(GLOB_RECURSE visual_estimator_files
    "src/visual_odometry/visual_estimator/*.cpp"
  )
  file(GLOB_RECURSE visual_loop_files
    "src/visual_odometry/visual_loop/*.cpp"
  )

  add_library(lvi_sam_camera_models STATIC ${camera_model_files})
  target_link_libraries(lvi_sam_camera_models ${OpenCV_LIBS} ${CERES_LIBRARIES})

  add_executable(visual_feature_node ${visual_feature_files})
  target_link_libraries(visual_feature_node lvi_sam_camera_models ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

  add_executable(visual_estimator_node ${visual_estimator_files})
  target_link_libraries(visual_estimator_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

  add_executable(visual_loop_node ${visual_loop_files})
  target_link_libraries(visual_loop_node lvi_sam_camera_models ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
endif()

# ============ 依赖绑定 ============
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ${common_dependencies})
ament_target_dependencies(${PROJECT_NAME}_mapOptimization ${common_dependencies})
if(BUILD_VISUAL)
  ament_target_dependencies(lvi_sam_camera_models ${visual_dependencies})
  ament_target_dependencies(visual_feature_node ${visual_dependencies})
  ament_target_dependencies(visual_estimator_node ${visual_dependencies})
  ament_target_dependencies(visual_loop_node ${visual_dependencies})
endif()

# ============ 安装 ============
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/gps_to_cartesian_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gps_to_cartesian_node
)

set(install_targets
  ${PROJECT_NAME}_imuPreintegration
  ${PROJECT_NAME}_mapOptimization
)
if(BUILD_VISUAL)
  list(APPEND install_targets
    visual_feature_node
    visual_estimator_node
    visual_loop_node
  )
endif()
install(TARGETS ${install_targets} DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
