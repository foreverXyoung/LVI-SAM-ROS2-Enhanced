#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#define PCL_NO_PRECOMPILE  // pcl include kdtree_flann throws error if PCL_NO_PRECOMPILE
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>
#include <stdexcept>

using namespace std;

typedef pcl::PointXYZI PointType;

using CloudType = pcl::PointCloud<PointType>;
using CloudPtr = pcl::PointCloud<PointType>::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

enum class SensorType { VELODYNE, OUSTER, LIVOX };

#include <chrono>
#define GET_TIME() std::chrono::high_resolution_clock::now()
#define GET_USED(t2, t1) std::chrono::duration<double>(t2 - t1).count()

struct CloudInfo {
    std_msgs::msg::Header header;

    std::vector<int32_t> start_ring_index, end_ring_index;

    std::vector<int32_t> point_col_ind;  // point column index in range image
    std::vector<float> point_range;      // point range

    int64_t imu_available, odom_available;

    // Attitude for LOAM initialization
    float imu_roll_init, imu_pitch_init, imu_yaw_init;

    // Initial guess from imu pre-integration
    float initial_guess_x, initial_guess_y, initial_guess_z;
    float initial_guess_roll, initial_guess_pitch, initial_guess_yaw;

    // Point cloud messages
    pcl::PointCloud<PointType>::Ptr cloud_deskewed{new pcl::PointCloud<PointType>};  // original cloud deskewed
    pcl::PointCloud<PointType>::Ptr cloud_corner{new pcl::PointCloud<PointType>};    // extracted corner feature
    pcl::PointCloud<PointType>::Ptr cloud_surface{new pcl::PointCloud<PointType>};   // extracted surface feature
};

inline rclcpp::NodeOptions makeInternalNodeOptions(
    const rclcpp::NodeOptions& parentOptions,
    const std::string& nodeName) {
    rclcpp::NodeOptions childOptions(parentOptions);
    auto arguments = childOptions.arguments();
    arguments.emplace_back("--ros-args");
    arguments.emplace_back("-r");
    arguments.emplace_back("__node:=" + nodeName);
    childOptions.arguments(arguments);
    return childOptions;
}

class ParamServer : public rclcpp::Node {
public:
    std::string robot_id;

    bool useRviz;

    // Topics
    string pointCloudTopic, imuTopic;
    string odomTopic, gpsTopic;

    // Frames
    string lidarFrame, baselinkFrame, odometryFrame, mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    bool useGpsFactor;
    bool gpsFactorAlwaysUse;
    float gpsCovThreshold, poseCovThreshold;
    float gpsInitialDistance, gpsFactorDistance, gpsVarianceFloor;
    float gpsTimeTolerance, gpsInnovationThreshold, gpsRobustKernelScale;
    bool gpsUseRobustNoise;
    int gpsQueueSize;
    string gpsExpectedFrame;

    // Optional non-GPS pose constraint, used by simulation odometry.
    bool useExternalPoseFactor;
    bool externalPoseOverride;
    string externalPoseTopic;
    float externalPosePositionVariance, externalPoseRotationVariance;

    // Keep the mapping odometry TF and the fused base TF independently
    // switchable.  On the real robot only TransformFusion owns
    // odom -> base_link; mapOptimization's legacy odom -> lidar_link is off.
    bool publishMappingOdomTF;
    bool publishFusedBaseTF;
    bool requireFreshLidarOdomForTF;
    bool publishImuPredictedOdomWhenUnlocalized;
    double maxLidarOdomAge;

    // Save pcd
    bool savePCD, saveKeyframeMap;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    SensorType sensor = SensorType::LIVOX;
    int N_SCAN, Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange, lidarMaxRange;
    bool selfFilterEnable;
    string selfFilterFrame;
    vector<double> selfFilterBoxMin, selfFilterBoxMax;

    // IMU
    float imuAccNoise, imuGyrNoise;
    float imuAccBiasN, imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    double imuAccelerationScale;
    // Sensor calibration (changes when the IMU changes or moves).
    // v_lidar = imuToLidarRotation * v_imu for acceleration and angular rate.
    Eigen::Matrix3d imuToLidarRotation;
    // LiDAR origin expressed in the raw IMU frame (metres).
    Eigen::Vector3d imuToLidarTranslation;
    // Right-side orientation correction: q_world_lidar =
    // q_world_imu * imuOrientationToLidarQuaternion.
    Eigen::Matrix3d imuOrientationToLidarRotation;
    Eigen::Quaterniond imuOrientationToLidarQuaternion;
    std::string imuOrientationSource;

    // Platform mounting calibration (does not change when only the IMU is
    // replaced). T_base_lidar also allows fused odom -> base_link output to be
    // composed without reading the TF tree.
    Eigen::Matrix3d baseToLidarRotation;
    Eigen::Vector3d baseToLidarTranslation;
    Eigen::Quaterniond baseToLidarQuaternion;
    bool baseToLidarConfigured = false;

    // LOAM
    float edgeThreshold, surfThreshold;
    int edgeFeatureMinValidNum, surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize, mappingSurfLeafSize;

    float z_tollerance, rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // Loop closure
    bool loopClosureEnableFlag;
    bool scanContextLoopEnableFlag;
    float scanContextDistanceThreshold;
    float loopClosureFrequency;
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;
    float externalLoopTimeTolerance;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    explicit ParamServer(
        std::string node_name,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node(node_name, options) {
        declare_and_get_parameter<bool>("useRviz", useRviz, true);

        declare_and_get_parameter<std::string>("pointCloudTopic", pointCloudTopic, "points");
        declare_and_get_parameter<std::string>("imuTopic", imuTopic, "imu/data");
        declare_and_get_parameter<std::string>("odomTopic", odomTopic, "/odometry/imu");
        declare_and_get_parameter<std::string>("gpsTopic", gpsTopic, "lio_sam/odometry/gps");

        declare_and_get_parameter<std::string>("lidarFrame", lidarFrame, "laser_data_frame");
        declare_and_get_parameter<std::string>("baselinkFrame", baselinkFrame, "base_link");
        declare_and_get_parameter<std::string>("odometryFrame", odometryFrame, "odom");
        declare_and_get_parameter<std::string>("mapFrame", mapFrame, "map");

        declare_and_get_parameter<bool>("useImuHeadingInitialization", useImuHeadingInitialization, false);
        declare_and_get_parameter<bool>("useGpsElevation", useGpsElevation, false);
        // Global factors are opt-in: a wrong frame or overconfident RTK source
        // can deform the whole pose graph.
        declare_and_get_parameter<bool>("useGpsFactor", useGpsFactor, false);
        declare_and_get_parameter<float>("gpsCovThreshold", gpsCovThreshold, 2.0);
        declare_and_get_parameter<float>("poseCovThreshold", poseCovThreshold, 25.0);
        declare_and_get_parameter<bool>("gpsFactorAlwaysUse", gpsFactorAlwaysUse, false);
        declare_and_get_parameter<float>("gpsInitialDistance", gpsInitialDistance, 5.0);
        declare_and_get_parameter<float>("gpsFactorDistance", gpsFactorDistance, 5.0);
        declare_and_get_parameter<float>("gpsVarianceFloor", gpsVarianceFloor, 1.0);
        declare_and_get_parameter<float>("gpsTimeTolerance", gpsTimeTolerance, 0.2);
        declare_and_get_parameter<float>("gpsInnovationThreshold", gpsInnovationThreshold, 10.0);
        declare_and_get_parameter<bool>("gpsUseRobustNoise", gpsUseRobustNoise, true);
        declare_and_get_parameter<float>("gpsRobustKernelScale", gpsRobustKernelScale, 2.0);
        declare_and_get_parameter<int>("gpsQueueSize", gpsQueueSize, 500);
        declare_and_get_parameter<std::string>("gpsExpectedFrame", gpsExpectedFrame, "");
        declare_and_get_parameter<bool>("useExternalPoseFactor", useExternalPoseFactor, false);
        declare_and_get_parameter<bool>("externalPoseOverride", externalPoseOverride, false);
        declare_and_get_parameter<std::string>("externalPoseTopic", externalPoseTopic, "/sim/local_odom");
        declare_and_get_parameter<float>("externalPosePositionVariance", externalPosePositionVariance, 1e-4);
        declare_and_get_parameter<float>("externalPoseRotationVariance", externalPoseRotationVariance, 1e-4);
        declare_and_get_parameter<bool>(
            "publishMappingOdomTF", publishMappingOdomTF, false);
        declare_and_get_parameter<bool>(
            "publishFusedBaseTF", publishFusedBaseTF, true);
        declare_and_get_parameter<bool>(
            "requireFreshLidarOdomForTF", requireFreshLidarOdomForTF, false);
        declare_and_get_parameter<bool>(
            "publishImuPredictedOdomWhenUnlocalized", publishImuPredictedOdomWhenUnlocalized, true);
        declare_and_get_parameter<double>(
            "maxLidarOdomAge", maxLidarOdomAge, 0.5);

        declare_and_get_parameter<bool>("savePCD", savePCD, false);
        declare_and_get_parameter<bool>("saveKeyframeMap", saveKeyframeMap, false);
        declare_and_get_parameter<std::string>("savePCDDirectory", savePCDDirectory, "/tmp/lvi_sam_maps/");

        std::string sensorStr;
        declare_and_get_parameter<std::string>("sensor", sensorStr, "ouster");
        if (sensorStr == "velodyne") {
            sensor = SensorType::VELODYNE;
        } else if (sensorStr == "ouster") {
            sensor = SensorType::OUSTER;
        } else if (sensorStr == "livox") {
            sensor = SensorType::LIVOX;
        } else {
            throw std::runtime_error(
                "Invalid sensor type '" + sensorStr +
                "' (expected velodyne, ouster, or livox)");
        }

        declare_and_get_parameter<int>("N_SCAN", N_SCAN, 64);
        declare_and_get_parameter<int>("Horizon_SCAN", Horizon_SCAN, 512);
        declare_and_get_parameter<int>("downsampleRate", downsampleRate, 1);
        declare_and_get_parameter<float>("lidarMinRange", lidarMinRange, 5.5);
        declare_and_get_parameter<float>("lidarMaxRange", lidarMaxRange, 1000.0);
        declare_and_get_parameter<bool>("selfFilterEnable", selfFilterEnable, false);
        declare_and_get_parameter<std::string>(
            "selfFilterFrame", selfFilterFrame, "lidar");
        declare_and_get_parameter<std::vector<double>>("selfFilterBoxMin", selfFilterBoxMin, std::vector<double>{-0.5, -0.5, -0.5});
        declare_and_get_parameter<std::vector<double>>("selfFilterBoxMax", selfFilterBoxMax, std::vector<double>{0.5, 0.5, 0.5});
        if (selfFilterFrame != "lidar" && selfFilterFrame != "base") {
            throw std::runtime_error(
                "selfFilterFrame must be 'lidar' or 'base'");
        }
        if (selfFilterBoxMin.size() != 3 || selfFilterBoxMax.size() != 3) {
            if (selfFilterEnable)
                throw std::runtime_error(
                    "enabled self filter requires three-value box min/max");
            RCLCPP_WARN(get_logger(),
                        "selfFilterBoxMin/selfFilterBoxMax should contain 3 values");
        } else if (selfFilterEnable) {
            for (std::size_t axis = 0; axis < 3; ++axis) {
                if (!std::isfinite(selfFilterBoxMin[axis]) ||
                    !std::isfinite(selfFilterBoxMax[axis]) ||
                    selfFilterBoxMin[axis] >= selfFilterBoxMax[axis]) {
                    throw std::runtime_error(
                        "self-filter box must be finite and min < max on every axis");
                }
            }
        }

        declare_and_get_parameter<float>("imuAccNoise", imuAccNoise, 9e-4);
        declare_and_get_parameter<float>("imuGyrNoise", imuGyrNoise, 1.6e-4);
        declare_and_get_parameter<float>("imuAccBiasN", imuAccBiasN, 5e-4);
        declare_and_get_parameter<float>("imuGyrBiasN", imuGyrBiasN, 7e-5);
        declare_and_get_parameter<float>("imuGravity", imuGravity, 9.80511);
        declare_and_get_parameter<float>("imuRPYWeight", imuRPYWeight, 0.01);
        declare_and_get_parameter<double>(
            "imuAccelerationScale", imuAccelerationScale, 1.0);

        const std::vector<double> identity{
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};

        // Legacy aliases remain readable so existing deployments do not stop
        // working. New profiles must use the descriptive parameter names.
        std::vector<double> legacyRotV, legacyRPYV, legacyTransV;
        declare_and_get_parameter<std::vector<double>>(
            "extrinsicRot", legacyRotV, identity);
        declare_and_get_parameter<std::vector<double>>(
            "extrinsicRPY", legacyRPYV, identity);
        declare_and_get_parameter<std::vector<double>>(
            "extrinsicTrans", legacyTransV, {0.0, 0.0, 0.0});

        std::vector<double> imuToLidarRotationV;
        std::vector<double> imuToLidarTranslationV;
        std::vector<double> imuOrientationToLidarRotationV;
        std::vector<double> baseToLidarRotationV;
        std::vector<double> baseToLidarTranslationV;
        declare_and_get_parameter<std::vector<double>>(
            "imuToLidarRotation", imuToLidarRotationV, {});
        declare_and_get_parameter<std::vector<double>>(
            "imuToLidarTranslation", imuToLidarTranslationV, {});
        declare_and_get_parameter<std::vector<double>>(
            "imuOrientationToLidarRotation",
            imuOrientationToLidarRotationV, {});
        declare_and_get_parameter<std::string>(
            "imuOrientationSource", imuOrientationSource, "message");
        declare_and_get_parameter<std::vector<double>>(
            "baseToLidarRotation", baseToLidarRotationV, {});
        declare_and_get_parameter<std::vector<double>>(
            "baseToLidarTranslation", baseToLidarTranslationV, {});

        const bool usingLegacyImuCalibration =
            imuToLidarRotationV.empty() ||
            imuToLidarTranslationV.empty() ||
            imuOrientationToLidarRotationV.empty();
        if (imuToLidarRotationV.empty()) imuToLidarRotationV = legacyRotV;
        if (imuToLidarTranslationV.empty()) imuToLidarTranslationV = legacyTransV;
        if (imuOrientationToLidarRotationV.empty()) {
            imuOrientationToLidarRotationV = legacyRPYV;
        }
        if (usingLegacyImuCalibration) {
            RCLCPP_WARN(
                get_logger(),
                "Using deprecated extrinsicRot/extrinsicRPY/extrinsicTrans "
                "fallback; migrate this IMU profile to imuToLidarRotation, "
                "imuOrientationToLidarRotation and imuToLidarTranslation");
        }

        if (imuToLidarRotationV.size() != 9 ||
            imuOrientationToLidarRotationV.size() != 9 ||
            imuToLidarTranslationV.size() != 3) {
            throw std::runtime_error(
                "imuToLidarRotation/imuOrientationToLidarRotation/"
                "imuToLidarTranslation must contain 9/9/3 values");
        }
        if (baseToLidarRotationV.empty() != baseToLidarTranslationV.empty()) {
            throw std::runtime_error(
                "baseToLidarRotation and baseToLidarTranslation must be "
                "configured together");
        }
        baseToLidarConfigured = !baseToLidarRotationV.empty();
        if (!baseToLidarConfigured) {
            baseToLidarRotationV = identity;
            baseToLidarTranslationV = {0.0, 0.0, 0.0};
        }
        if (baseToLidarRotationV.size() != 9 ||
            baseToLidarTranslationV.size() != 3) {
            throw std::runtime_error(
                "baseToLidarRotation/baseToLidarTranslation must contain "
                "9/3 values");
        }

        imuToLidarRotation = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            imuToLidarRotationV.data(), 3, 3);
        imuOrientationToLidarRotation = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            imuOrientationToLidarRotationV.data(), 3, 3);
        imuToLidarTranslation = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            imuToLidarTranslationV.data(), 3, 1);
        baseToLidarRotation = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            baseToLidarRotationV.data(), 3, 3);
        baseToLidarTranslation = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            baseToLidarTranslationV.data(), 3, 1);
        const auto validateRotation = [](const Eigen::Matrix3d& rotation,
                                         const char* parameterName) {
            if (!rotation.allFinite() ||
                !(rotation.transpose() * rotation)
                     .isApprox(Eigen::Matrix3d::Identity(), 1e-3) ||
                std::abs(rotation.determinant() - 1.0) > 1e-3) {
                throw std::runtime_error(
                    std::string(parameterName) +
                    " must be a finite orthonormal 3x3 rotation matrix");
            }
        };
        validateRotation(imuToLidarRotation, "imuToLidarRotation");
        validateRotation(
            imuOrientationToLidarRotation,
            "imuOrientationToLidarRotation");
        validateRotation(baseToLidarRotation, "baseToLidarRotation");
        if (!imuToLidarTranslation.allFinite() ||
            !baseToLidarTranslation.allFinite()) {
            throw std::runtime_error(
                "IMU/LiDAR calibration translations must contain finite values");
        }
        if (imuOrientationSource != "message" &&
            imuOrientationSource != "mount") {
            throw std::runtime_error(
                "imuOrientationSource must be 'message' or 'mount'");
        }
        if (imuOrientationSource == "mount" && !baseToLidarConfigured) {
            throw std::runtime_error(
                "imuOrientationSource=mount requires a base-to-LiDAR "
                "mounting profile");
        }
        if (imuOrientationSource == "mount" && imuRPYWeight > 0.0f) {
            throw std::runtime_error(
                "imuOrientationSource=mount requires imuRPYWeight=0 because "
                "a fixed mounting attitude is not a dynamic IMU observation");
        }
        imuOrientationToLidarQuaternion =
            Eigen::Quaterniond(imuOrientationToLidarRotation);
        baseToLidarQuaternion = Eigen::Quaterniond(baseToLidarRotation);
        imuOrientationToLidarQuaternion.normalize();
        baseToLidarQuaternion.normalize();
        if (selfFilterEnable && selfFilterFrame == "base" &&
            !baseToLidarConfigured) {
            throw std::runtime_error(
                "selfFilterFrame=base requires baseToLidarRotation and "
                "baseToLidarTranslation");
        }
        if (selfFilterEnable) {
            RCLCPP_INFO(
                get_logger(),
                "Self-filter enabled in %s frame: min=[%.3f %.3f %.3f], "
                "max=[%.3f %.3f %.3f]",
                selfFilterFrame.c_str(),
                selfFilterBoxMin[0], selfFilterBoxMin[1],
                selfFilterBoxMin[2], selfFilterBoxMax[0],
                selfFilterBoxMax[1], selfFilterBoxMax[2]);
        }

        declare_and_get_parameter<float>("edgeThreshold", edgeThreshold, 1.0);
        declare_and_get_parameter<float>("surfThreshold", surfThreshold, 0.1);
        declare_and_get_parameter<int>("edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        declare_and_get_parameter<int>("surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        declare_and_get_parameter<float>("odometrySurfLeafSize", odometrySurfLeafSize, 0.4);
        declare_and_get_parameter<float>("mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        declare_and_get_parameter<float>("mappingSurfLeafSize", mappingSurfLeafSize, 0.4);

        declare_and_get_parameter<float>("z_tollerance", z_tollerance, 1000.0);
        declare_and_get_parameter<float>("rotation_tollerance", rotation_tollerance, 1000.0);

        declare_and_get_parameter<int>("numberOfCores", numberOfCores, 4);
        declare_and_get_parameter<double>("mappingProcessInterval", mappingProcessInterval, 0.15);

        declare_and_get_parameter<float>("surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        declare_and_get_parameter<float>("surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        declare_and_get_parameter<float>("surroundingKeyframeDensity", surroundingKeyframeDensity, 2.0);
        declare_and_get_parameter<float>("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        declare_and_get_parameter<bool>("loopClosureEnableFlag", loopClosureEnableFlag, true);
        declare_and_get_parameter<bool>("scanContextLoopEnableFlag", scanContextLoopEnableFlag, false);
        declare_and_get_parameter<float>("scanContextDistanceThreshold", scanContextDistanceThreshold, 0.3);
        declare_and_get_parameter<float>("loopClosureFrequency", loopClosureFrequency, 1.0);
        declare_and_get_parameter<int>("surroundingKeyframeSize", surroundingKeyframeSize, 50);
        declare_and_get_parameter<float>("historyKeyframeSearchRadius", historyKeyframeSearchRadius, 15.0);
        declare_and_get_parameter<float>("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        declare_and_get_parameter<int>("historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        declare_and_get_parameter<float>("historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);
        declare_and_get_parameter<float>("externalLoopTimeTolerance", externalLoopTimeTolerance, 0.2);

        declare_and_get_parameter<float>("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1000.0);
        declare_and_get_parameter<float>("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        declare_and_get_parameter<float>("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        if (pointCloudTopic.empty() || imuTopic.empty() || odomTopic.empty() ||
            lidarFrame.empty() || baselinkFrame.empty() ||
            odometryFrame.empty() || mapFrame.empty()) {
            throw std::runtime_error(
                "LiDAR/IMU/odometry topics and frame names must not be empty");
        }
        if (useGpsFactor && gpsTopic.empty()) {
            throw std::runtime_error(
                "gpsTopic must not be empty when GPS factors are enabled");
        }
        if ((savePCD || saveKeyframeMap) && savePCDDirectory.empty()) {
            throw std::runtime_error(
                "savePCDDirectory must not be empty when map saving is enabled");
        }
        if (N_SCAN <= 0 || Horizon_SCAN <= 0 || downsampleRate <= 0) {
            throw std::runtime_error(
                "N_SCAN, Horizon_SCAN, and downsampleRate must be positive");
        }
        if (!std::isfinite(lidarMinRange) || !std::isfinite(lidarMaxRange) ||
            lidarMinRange < 0.0f || lidarMaxRange <= lidarMinRange) {
            throw std::runtime_error(
                "lidarMaxRange must be greater than non-negative lidarMinRange");
        }
        if (!std::isfinite(imuAccNoise) || !std::isfinite(imuGyrNoise) ||
            !std::isfinite(imuAccBiasN) || !std::isfinite(imuGyrBiasN) ||
            !std::isfinite(imuGravity) || !std::isfinite(imuRPYWeight) ||
            !std::isfinite(imuAccelerationScale) ||
            imuAccNoise <= 0.0f || imuGyrNoise <= 0.0f ||
            imuAccBiasN <= 0.0f || imuGyrBiasN <= 0.0f ||
            imuGravity <= 0.0f || imuRPYWeight < 0.0f ||
            imuAccelerationScale <= 0.0) {
            throw std::runtime_error(
                "IMU noise, bias random walk, gravity, and acceleration "
                "scale parameters must be positive");
        }
        if (!std::isfinite(edgeThreshold) || !std::isfinite(surfThreshold) ||
            edgeThreshold <= 0.0f || surfThreshold <= 0.0f ||
            edgeFeatureMinValidNum < 0 || surfFeatureMinValidNum < 0) {
            throw std::runtime_error(
                "feature thresholds must be positive and minimum feature "
                "counts must be non-negative");
        }
        if (!std::isfinite(odometrySurfLeafSize) ||
            !std::isfinite(mappingCornerLeafSize) ||
            !std::isfinite(mappingSurfLeafSize) ||
            odometrySurfLeafSize <= 0.0f || mappingCornerLeafSize <= 0.0f ||
            mappingSurfLeafSize <= 0.0f) {
            throw std::runtime_error("voxel leaf sizes must be positive");
        }
        if (!std::isfinite(z_tollerance) ||
            !std::isfinite(rotation_tollerance) || z_tollerance < 0.0f ||
            rotation_tollerance < 0.0f) {
            throw std::runtime_error(
                "translation and rotation tolerances must be finite and non-negative");
        }
        if (numberOfCores <= 0 || !std::isfinite(mappingProcessInterval) ||
            mappingProcessInterval < 0.0) {
            throw std::runtime_error(
                "numberOfCores must be positive and mappingProcessInterval non-negative");
        }
        if (useExternalPoseFactor &&
            (!std::isfinite(externalPosePositionVariance) ||
             !std::isfinite(externalPoseRotationVariance) ||
             externalPosePositionVariance <= 0.0f ||
             externalPoseRotationVariance <= 0.0f)) {
            throw std::runtime_error(
                "enabled external pose factors require positive variances");
        }
        if (!std::isfinite(maxLidarOdomAge) || maxLidarOdomAge <= 0.0) {
            throw std::runtime_error("maxLidarOdomAge must be positive");
        }
        if (!std::isfinite(gpsCovThreshold) ||
            !std::isfinite(poseCovThreshold) ||
            !std::isfinite(gpsInitialDistance) ||
            !std::isfinite(gpsFactorDistance) ||
            !std::isfinite(gpsVarianceFloor) ||
            !std::isfinite(gpsTimeTolerance) ||
            !std::isfinite(gpsInnovationThreshold) ||
            !std::isfinite(gpsRobustKernelScale) || gpsCovThreshold <= 0.0f ||
            poseCovThreshold <= 0.0f || gpsInitialDistance < 0.0f ||
            gpsFactorDistance < 0.0f || gpsVarianceFloor <= 0.0f ||
            gpsTimeTolerance <= 0.0f || gpsInnovationThreshold <= 0.0f ||
            gpsRobustKernelScale <= 0.0f || gpsQueueSize <= 0) {
            throw std::runtime_error(
                "GPS covariance/time/innovation parameters must be finite and valid");
        }
        if (!std::isfinite(surroundingkeyframeAddingDistThreshold) ||
            !std::isfinite(surroundingkeyframeAddingAngleThreshold) ||
            !std::isfinite(surroundingKeyframeDensity) ||
            !std::isfinite(surroundingKeyframeSearchRadius) ||
            surroundingkeyframeAddingDistThreshold <= 0.0f ||
            surroundingkeyframeAddingAngleThreshold <= 0.0f ||
            surroundingKeyframeDensity <= 0.0f ||
            surroundingKeyframeSearchRadius <= 0.0f) {
            throw std::runtime_error(
                "surrounding-keyframe thresholds must be finite and positive");
        }
        if (loopClosureEnableFlag &&
            (!std::isfinite(loopClosureFrequency) ||
             !std::isfinite(historyKeyframeSearchRadius) ||
             !std::isfinite(historyKeyframeSearchTimeDiff) ||
             !std::isfinite(historyKeyframeFitnessScore) ||
             !std::isfinite(externalLoopTimeTolerance) ||
             loopClosureFrequency <= 0.0f || surroundingKeyframeSize <= 0 ||
             historyKeyframeSearchRadius <= 0.0f ||
             historyKeyframeSearchTimeDiff <= 0.0f ||
             historyKeyframeSearchNum <= 0 ||
             historyKeyframeFitnessScore <= 0.0f ||
             externalLoopTimeTolerance <= 0.0f)) {
            throw std::runtime_error(
                "enabled loop closure requires positive frequency/search/fitness parameters");
        }
        if (!std::isfinite(scanContextDistanceThreshold) ||
            scanContextDistanceThreshold <= 0.0f ||
            scanContextDistanceThreshold >= 1.0f) {
            throw std::runtime_error(
                "scanContextDistanceThreshold must be in (0, 1)");
        }
        if (!std::isfinite(globalMapVisualizationSearchRadius) ||
            !std::isfinite(globalMapVisualizationPoseDensity) ||
            !std::isfinite(globalMapVisualizationLeafSize) ||
            globalMapVisualizationSearchRadius <= 0.0f ||
            globalMapVisualizationPoseDensity <= 0.0f ||
            globalMapVisualizationLeafSize <= 0.0f) {
            throw std::runtime_error(
                "global-map visualization parameters must be finite and positive");
        }

        usleep(100);
    }

    template <typename T>
    void declare_and_get_parameter(const std::string& name, T& variable, const T& default_value) {
        this->declare_parameter<T>(name, default_value);
        this->get_parameter(name, variable);
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in) {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc *= imuAccelerationScale;

        const double accelerationNorm = acc.norm();
        if (std::isfinite(accelerationNorm) &&
            (accelerationNorm < 0.25 * imuGravity ||
             accelerationNorm > 4.0 * imuGravity)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Scaled IMU acceleration norm %.3f m/s^2 is far from gravity; "
                "verify imu_source and imuAccelerationScale=%.6f",
                accelerationNorm, imuAccelerationScale);
        }

        acc = imuToLidarRotation * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = imuToLidarRotation * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        // Orientation and vector calibration are deliberately independent:
        // some drivers publish vectors and attitude in different conventions.
        double q_norm = sqrt(q_from.x() * q_from.x() + q_from.y() * q_from.y() + q_from.z() * q_from.z() + q_from.w() * q_from.w());
        Eigen::Quaterniond q_final;
        if (imuOrientationSource == "mount") {
            // The platform is assumed level when the estimator starts. This
            // is intended for IMUs such as MID-360 whose driver publishes no
            // usable attitude quaternion.
            q_final = baseToLidarQuaternion;
        } else if (!std::isfinite(q_norm) || q_norm < 0.1) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "IMU orientation invalid (norm=%.3f); using only the "
                "configured IMU-to-LiDAR orientation correction",
                q_norm);
            q_final = imuOrientationToLidarQuaternion;
        } else {
            q_final = q_from * imuOrientationToLidarQuaternion;
        }
        q_final.normalize();
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        return imu_out;
    }

    bool isPointInSelfFilterBox(const PointType& point) const {
        if (!selfFilterEnable || selfFilterBoxMin.size() != 3 || selfFilterBoxMax.size() != 3) return false;

        Eigen::Vector3d testPoint(point.x, point.y, point.z);
        if (selfFilterFrame == "base") {
            // Point clouds arrive in lidarFrame. Apply the configured
            // T_base_lidar directly; TF is not consulted by the estimator.
            testPoint =
                baseToLidarRotation * testPoint + baseToLidarTranslation;
        }
        return testPoint.x() >= selfFilterBoxMin[0] &&
               testPoint.x() <= selfFilterBoxMax[0] &&
               testPoint.y() >= selfFilterBoxMin[1] &&
               testPoint.y() <= selfFilterBoxMax[1] &&
               testPoint.z() >= selfFilterBoxMin[2] &&
               testPoint.z() <= selfFilterBoxMax[2];
    }
};

inline sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub,
                                                  pcl::PointCloud<PointType>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame) {
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0) thisPub->publish(tempCloud);
    return tempCloud;
}

template <typename T>
double stamp2Sec(const T& stamp) {
    return rclcpp::Time(stamp).seconds();
}

inline float pointDistance(PointType p) { return sqrt(p.x * p.x + p.y * p.y + p.z * p.z); }

inline float pointDistance(PointType p1, PointType p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

inline rmw_qos_profile_t qos_profile{RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                     1,
                                     RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                     RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                     RMW_QOS_DEADLINE_DEFAULT,
                                     RMW_QOS_LIFESPAN_DEFAULT,
                                     RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                     RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                     false};

inline auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

inline rmw_qos_profile_t qos_profile_imu{RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                         2000,
                                         RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                         RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                         RMW_QOS_DEADLINE_DEFAULT,
                                         RMW_QOS_LIFESPAN_DEFAULT,
                                         RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                         RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                         false};

inline auto qos_imu = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_imu.history, qos_profile_imu.depth), qos_profile_imu);

inline rmw_qos_profile_t qos_profile_lidar{RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                           5,
                                           RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                           RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                           RMW_QOS_DEADLINE_DEFAULT,
                                           RMW_QOS_LIFESPAN_DEFAULT,
                                           RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                           RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                           false};

inline auto qos_lidar = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_lidar.history, qos_profile_lidar.depth), qos_profile_lidar);

#endif
