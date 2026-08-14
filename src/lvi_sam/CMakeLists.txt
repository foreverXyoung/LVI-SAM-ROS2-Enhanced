cmake_minimum_required(VERSION 3.12)
project(lvi_sam)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

option(BUILD_VISUAL "Build the camera/VINS/DBoW2 pipeline" ON)

# ROS interfaces used by the C++ targets.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(lvi_sam_msgs REQUIRED)

# Native libraries. The package-local image adapter deliberately avoids
# cv_bridge so each executable resolves one OpenCV ABI selected by this find.
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM 4.0 REQUIRED)
if(GTSAM_VERSION VERSION_LESS 4.0 OR
   NOT GTSAM_VERSION VERSION_LESS 5.0)
  message(FATAL_ERROR
    "LVI-SAM requires GTSAM >= 4.0 and < 5.0; found ${GTSAM_VERSION}")
endif()
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
if(BUILD_VISUAL)
  find_package(Ceres REQUIRED)
  find_package(Boost REQUIRED)
endif()

message(STATUS "LVI-SAM BUILD_VISUAL=${BUILD_VISUAL}")
message(STATUS "LVI-SAM OpenCV ${OpenCV_VERSION}: ${OpenCV_DIR}")
message(STATUS "LVI-SAM GTSAM ${GTSAM_VERSION}: ${GTSAM_DIR}")

# Some upstream PCL/GTSAM releases still expose directory variables instead
# of complete imported targets. Keep these compatibility paths centralized.
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

set(lidar_dependencies
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
  livox_ros_driver2
  lvi_sam_msgs
)

set(visual_dependencies
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# LiDAR-inertial subsystem.
add_executable(${PROJECT_NAME}_imuPreintegration
  src/lidar_odometry/imuPreintegration.cpp
)
target_link_libraries(${PROJECT_NAME}_imuPreintegration
  gtsam
  Eigen3::Eigen
  Threads::Threads
)
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ${lidar_dependencies})

add_executable(${PROJECT_NAME}_mapOptimization
  src/lidar_odometry/mapOptmization.cpp
  include/sc/Scancontext.cpp
)
target_link_libraries(${PROJECT_NAME}_mapOptimization
  gtsam
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
  Threads::Threads
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_mapOptimization ${lidar_dependencies})

# Optional visual-inertial subsystem.
if(BUILD_VISUAL)
  file(GLOB_RECURSE camera_model_files CONFIGURE_DEPENDS
    "src/visual_odometry/visual_feature/camera_models/*.cc"
  )
  file(GLOB_RECURSE visual_feature_files CONFIGURE_DEPENDS
    "src/visual_odometry/visual_feature/*.cpp"
  )
  file(GLOB_RECURSE visual_estimator_files CONFIGURE_DEPENDS
    "src/visual_odometry/visual_estimator/*.cpp"
  )
  file(GLOB_RECURSE visual_loop_files CONFIGURE_DEPENDS
    "src/visual_odometry/visual_loop/*.cpp"
  )

  add_library(lvi_sam_camera_models STATIC ${camera_model_files})
  target_link_libraries(lvi_sam_camera_models
    Eigen3::Eigen
    Threads::Threads
    ${OpenCV_LIBS}
    Ceres::ceres
    Boost::boost
  )
  ament_target_dependencies(lvi_sam_camera_models rclcpp)

  add_executable(visual_feature_node ${visual_feature_files})
  target_link_libraries(visual_feature_node
    lvi_sam_camera_models
    Eigen3::Eigen
    Threads::Threads
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Ceres::ceres
  )
  ament_target_dependencies(visual_feature_node ${visual_dependencies})

  add_executable(visual_estimator_node ${visual_estimator_files})
  target_link_libraries(visual_estimator_node
    Eigen3::Eigen
    Threads::Threads
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Ceres::ceres
  )
  ament_target_dependencies(visual_estimator_node ${visual_dependencies})

  add_executable(visual_loop_node ${visual_loop_files})
  target_link_libraries(visual_loop_node
    lvi_sam_camera_models
    Eigen3::Eigen
    Threads::Threads
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Ceres::ceres
    Boost::boost
  )
  ament_target_dependencies(visual_loop_node ${visual_dependencies})
endif()

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(config_test_arguments
    --config-dir ${CMAKE_CURRENT_SOURCE_DIR}/config
  )
  if(NOT BUILD_VISUAL)
    list(APPEND config_test_arguments --lidar-only)
  endif()
  add_test(
    NAME test_config_contracts
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_config.py
      ${config_test_arguments}
  )

  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    if(BUILD_VISUAL)
      ament_add_gtest(test_image_conversion test/test_image_conversion.cpp)
      if(TARGET test_image_conversion)
        ament_target_dependencies(test_image_conversion sensor_msgs std_msgs)
        target_link_libraries(test_image_conversion ${OpenCV_LIBS})
      endif()
      ament_add_gtest(
        test_visual_frame_conventions
        test/test_visual_frame_conventions.cpp
      )
      if(TARGET test_visual_frame_conventions)
        target_link_libraries(test_visual_frame_conventions Eigen3::Eigen)
      endif()
    endif()
    ament_add_gtest(test_topic_names test/test_topic_names.cpp)
    ament_add_gtest(
      test_internal_odom_metadata
      test/test_internal_odom_metadata.cpp
    )
    ament_add_gtest(
      test_localization_status_contract
      test/test_localization_status_contract.cpp
    )
    if(TARGET test_localization_status_contract)
      ament_target_dependencies(test_localization_status_contract lvi_sam_msgs)
    endif()
    ament_add_gtest(
      test_localization_status_projection
      test/test_localization_status_projection.cpp
    )
    if(TARGET test_localization_status_projection)
      ament_target_dependencies(test_localization_status_projection lvi_sam_msgs)
    endif()
    ament_add_gtest(
      test_localization_reset_contract
      test/test_localization_reset_contract.cpp
    )
    if(TARGET test_localization_reset_contract)
      ament_target_dependencies(test_localization_reset_contract lvi_sam_msgs)
    endif()
  else()
    message(WARNING "ament_cmake_gtest not found; C++ unit tests are skipped")
  endif()
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

install(
  PROGRAMS scripts/gps_to_cartesian_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gps_to_cartesian_node
)
install(
  PROGRAMS scripts/verify_localization_status.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME verify_localization_status
)
install(
  PROGRAMS scripts/validate_config.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lvi_sam_validate_config
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
ament_export_dependencies(ament_index_cpp sensor_msgs std_msgs lvi_sam_msgs)
ament_package()
