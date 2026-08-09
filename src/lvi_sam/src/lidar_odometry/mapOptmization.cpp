#include "utility.hpp"
#include "file_tools.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <limits>
#include <map>
#include <pcl/common/angles.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <opencv2/opencv.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "imageProjection.hpp"
#include "featureExtraction.hpp"

using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G;  // GPS pose
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
struct PointXYZIRPYT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                     // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(
                                                     float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

class mapOptimization : public ParamServer {
public:
    template <typename PointT>
    bool savePCDIfNotEmpty(const std::string& path,
                           const pcl::PointCloud<PointT>& cloud,
                           bool binary = true) {
        if (cloud.empty()) {
            RCLCPP_WARN(get_logger(),
                        "Skipping empty PCD output: %s", path.c_str());
            // Truncate any file left by an earlier mapping run so localization
            // cannot accidentally load stale features for this keyframe index.
            std::ofstream emptyMarker(path, std::ios::out | std::ios::trunc);
            if (!emptyMarker.good()) {
                RCLCPP_ERROR(get_logger(),
                             "Failed to clear stale PCD output: %s", path.c_str());
            }
            return false;
        }

        try {
            const int result = binary
                ? pcl::io::savePCDFileBinary(path, cloud)
                : pcl::io::savePCDFileASCII(path, cloud);
            if (result < 0) {
                RCLCPP_ERROR(get_logger(), "Failed to save PCD: %s", path.c_str());
                return false;
            }
        } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "Failed to save PCD %s: %s",
                         path.c_str(), error.what());
            return false;
        }
        return true;
    }

    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2* isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;

    // rclcpp::Service<lio_sam::srv::SaveMap>::SharedPtr srvSaveMap;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subCloud;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGPS;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subExternalPose;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subLoop;

    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    std::deque<nav_msgs::msg::Odometry> externalPoseQueue;
    nav_msgs::msg::Odometry latestExternalPose;
    bool latestExternalPoseAvailable = false;
    CloudInfo cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> copyCornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> copySurfCloudKeyFrames;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;    // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;      // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;  // downsampled corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;    // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec;  // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec;  // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;  // for surrounding key poses of scan-to-map optimization

    rclcpp::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;
    std::mutex mtxExternalPose;
    std::mutex mtxScanContext;
    std::mutex mtxGPS;

    bool isDegenerate = false;
    Eigen::Matrix<float, 6, 6> matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer;  // from new to old
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::SharedNoiseModel> loopNoiseQueue;
    deque<std_msgs::msg::Float64MultiArray> loopInfoVec;

    nav_msgs::msg::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    FeatureExtraction featureExtraction;
    ImageProjection imageProjection;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPoints;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfacePoints;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDynamicFilterKeptPoints;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDynamicFilterRejectedPoints;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubDynamicFilterKeepRatio;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubLocalizationState;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srvForceRelocalize;

    /////////////////////////////////// SC Start ///////////////////////////////////
    SCManager scManager;
    pcl::PointCloud<PointType>::Ptr laserCloudRaw{new pcl::PointCloud<PointType>()};    // giseop
    pcl::PointCloud<PointType>::Ptr laserCloudRawDS{new pcl::PointCloud<PointType>()};  // giseop
    /////////////////////////////////// SC End ///////////////////////////////////

    explicit mapOptimization(const rclcpp::NodeOptions& options)
        : ParamServer("mapOptimizationParamServer", options) {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        pubKeyPoses = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/trajectory", 1);
        pubLaserCloudSurround = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>("lio_sam/mapping/odometry", qos);
        pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>("lio_sam/mapping/odometry_incremental", qos);
        pubPath = create_publisher<nav_msgs::msg::Path>("lio_sam/mapping/path", 1);
        br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        subImu = create_subscription<sensor_msgs::msg::Imu>(imuTopic, qos_imu,
                                                            [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imageProjection.imuHandler(msg); });
        subOdom = create_subscription<nav_msgs::msg::Odometry>(odomTopic + "_incremental", qos_imu,
                                                               [this](const nav_msgs::msg::Odometry::SharedPtr msg) { imageProjection.odometryHandler(msg); });

        subCloud = create_subscription<livox_ros_driver2::msg::CustomMsg>(pointCloudTopic, qos_lidar,
                                                                          std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
        if (useExternalPoseFactor) {
            subExternalPose = create_subscription<nav_msgs::msg::Odometry>(
                externalPoseTopic, 200,
                std::bind(&mapOptimization::externalPoseHandler, this,
                          std::placeholders::_1));
        }
        subLoop = create_subscription<std_msgs::msg::Float64MultiArray>("lio_loop/loop_closure_detection", qos,
                                                                        std::bind(&mapOptimization::loopInfoHandler, this, std::placeholders::_1));

        // srvSaveMap = create_service<lio_sam::srv::SaveMap>("lio_sam/save_map", saveMapService);
        pubHistoryKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
        pubLoopConstraintEdge = create_publisher<visualization_msgs::msg::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);

        pubRecentKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/map_local", 1);
        pubRecentKeyFrame = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);

        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/deskew/cloud_deskewed", 1);
        pubCornerPoints = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/feature/cloud_corner", 1);
        pubSurfacePoints = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/feature/cloud_surface", 1);
        pubDynamicFilterKeptPoints =
            create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/localization/dynamic_filter/kept_points", 1);
        pubDynamicFilterRejectedPoints =
            create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/localization/dynamic_filter/rejected_points", 1);
        pubDynamicFilterKeepRatio =
            create_publisher<std_msgs::msg::Float32>("lio_sam/localization/dynamic_filter/keep_ratio", 1);
        pubLocalizationState =
            create_publisher<std_msgs::msg::String>("lio_sam/localization/state", 1);

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity,
                                                      surroundingKeyframeDensity);  // for surrounding key poses of scan-to-map optimization

        allocateMemory();
        scManager.SC_DIST_THRES = scanContextDistanceThreshold;

        /////////////////////////////////// Sc & Loc  ///////////////////////////////////

        if (!savePCDDirectory.empty() && savePCDDirectory[0] != '/') {
            const char* home = std::getenv("HOME");
            if (home == nullptr) {
                throw std::runtime_error(
                    "HOME is not set; savePCDDirectory must be absolute");
            }
            savePCDDirectory = std::string(home) + "/" + savePCDDirectory;
        }
        if (!savePCDDirectory.empty() && savePCDDirectory.back() != '/')
            savePCDDirectory += '/';
        if (savePCD) {
            if (savePCDDirectory.empty()) {
                throw std::runtime_error("savePCD is enabled but savePCDDirectory is empty");
            }
            std::cout << "save pcd to: " << savePCDDirectory << std::endl;
            if (!createDirectoryIfNotExists(savePCDDirectory) ||
                !createDirectoryIfNotExists(savePCDDirectory + "Scans/") ||
                !createDirectoryIfNotExists(savePCDDirectory + "SCDs/") ||
                !createDirectoryIfNotExists(savePCDDirectory + "SurfMap/") ||
                !createDirectoryIfNotExists(savePCDDirectory + "CornerMap/")) {
                throw std::runtime_error(
                    "Failed to prepare LIO-SAM PCD output directory: " +
                    savePCDDirectory);
            }
        }

        InitLocationMode();

        if (gpsCovThreshold <= 0.0f || gpsVarianceFloor <= 0.0f) {
            throw std::runtime_error(
                "gpsCovThreshold and gpsVarianceFloor must be positive");
        }
        gpsTimeTolerance = std::max(0.01f, gpsTimeTolerance);
        gpsQueueSize = std::max(1, gpsQueueSize);
        if (useRTKInitialization && !useRTKAssist) {
            throw std::runtime_error(
                "Loc.useRTKInitialization requires Loc.useRTKAssist=true");
        }
        if (rtkUseHeading && rtkYawVarianceThreshold <= 0.0f) {
            throw std::runtime_error(
                "Loc.rtkYawVarianceThreshold must be positive when RTK heading is enabled");
        }

        // Mapping GPS factors and localization RTK assistance are independent
        // consumers of the same map-aligned odometry interface. Subscribe when
        // either feature is enabled; do not make RTK initialization depend on
        // useGpsFactor.
        if (useGpsFactor || useRTKAssist) {
            const std::string expectedFrame =
                LocEnableFlag && !rtkExpectedFrame.empty()
                    ? rtkExpectedFrame
                    : gpsExpectedFrame;
            if (expectedFrame.empty()) {
                RCLCPP_WARN(
                    get_logger(),
                    "RTK/GPS fusion is enabled without an expected frame; "
                    "set gpsExpectedFrame or Loc.rtkExpectedFrame");
            }
            RCLCPP_INFO(
                get_logger(),
                "RTK/GPS input: topic=%s frame=%s GPS-factor=%s localization-assist=%s covariance<=%.3f time-tolerance=%.3f s",
                gpsTopic.c_str(),
                expectedFrame.empty() ? "<unchecked>" : expectedFrame.c_str(),
                useGpsFactor ? "on" : "off",
                useRTKAssist ? "on" : "off",
                gpsCovThreshold, gpsTimeTolerance);
            subGPS = create_subscription<nav_msgs::msg::Odometry>(
                gpsTopic, rclcpp::SensorDataQoS().keep_last(200),
                std::bind(&mapOptimization::gpsHandler, this,
                          std::placeholders::_1));
        }

        srvForceRelocalize = create_service<std_srvs::srv::Trigger>(
            "/lio_sam/localization/force_relocalize",
            std::bind(
                &mapOptimization::forceRelocalizeHandler,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
    }

    void allocateMemory() {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());    // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());      // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());  // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());    // downsampled surf featuer set from odoOptimization

        laserCloudCornerLastDSFiltered.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLastDSFiltered.reset(new pcl::PointCloud<PointType>());
        dynamicFilterKeptPoints.reset(new pcl::PointCloud<PointType>());
        dynamicFilterRejectedPoints.reset(new pcl::PointCloud<PointType>());

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Ho…28715 tokens truncated…        gps_x - transformTobeMapped[3],
                    gps_y - transformTobeMapped[4]);
                if (gpsInnovationThreshold > 0.0f &&
                    positionInnovation > gpsInnovationThreshold) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Rejecting GPS factor: %.3f m innovation exceeds %.3f m",
                        positionInnovation, gpsInnovationThreshold);
                    continue;
                }

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (gpsFactorDistance > 0.0f && hasLastGPSPoint &&
                    pointDistance(curGPSPoint, lastGPSPoint) < gpsFactorDistance)
                    continue;
                else {
                    lastGPSPoint = curGPSPoint;
                    hasLastGPSPoint = true;
                }

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, gpsVarianceFloor),
                           max(noise_y, gpsVarianceFloor),
                           max(noise_z, gpsVarianceFloor);
                noiseModel::Base::shared_ptr gps_noise =
                    noiseModel::Diagonal::Variances(Vector3);
                if (gpsUseRobustNoise) {
                    gps_noise = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::Huber::Create(
                            std::max(0.1f, gpsRobustKernelScale)),
                        gps_noise);
                }
                gtsam::GPSFactor gps_factor(
                    cloudKeyPoses3D->size(),
                    gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addExternalPoseFactor() {
        if (!useExternalPoseFactor || cloudKeyPoses3D->points.empty()) {
            return;
        }
        std::lock_guard<std::mutex> lock(mtxExternalPose);
        if (externalPoseQueue.empty()) return;

        while (!externalPoseQueue.empty()) {
            const double poseTime =
                stamp2Sec(externalPoseQueue.front().header.stamp);
            if (poseTime < timeLaserInfoCur - 0.2) {
                externalPoseQueue.pop_front();
                continue;
            }
            if (poseTime > timeLaserInfoCur + 0.2) return;

            const nav_msgs::msg::Odometry external =
                externalPoseQueue.front();
            externalPoseQueue.pop_front();
            const auto& position = external.pose.pose.position;
            const auto& orientation = external.pose.pose.orientation;
            if (!std::isfinite(position.x) ||
                !std::isfinite(position.y) ||
                !std::isfinite(position.z) ||
                !std::isfinite(orientation.x) ||
                !std::isfinite(orientation.y) ||
                !std::isfinite(orientation.z) ||
                !std::isfinite(orientation.w)) {
                continue;
            }
            const double quaternionNorm = std::sqrt(
                orientation.x * orientation.x +
                orientation.y * orientation.y +
                orientation.z * orientation.z +
                orientation.w * orientation.w);
            if (quaternionNorm < 1e-6) continue;

            const gtsam::Rot3 externalRotation = gtsam::Rot3::Quaternion(
                orientation.w / quaternionNorm,
                orientation.x / quaternionNorm,
                orientation.y / quaternionNorm,
                orientation.z / quaternionNorm);
            const gtsam::Pose3 externalPose(
                externalRotation,
                gtsam::Point3(position.x, position.y, position.z));
            gtsam::Vector poseVariances(6);
            poseVariances <<
                max(externalPoseRotationVariance, 1e-9f),
                max(externalPoseRotationVariance, 1e-9f),
                max(externalPoseRotationVariance, 1e-9f),
                max(externalPosePositionVariance, 1e-9f),
                max(externalPosePositionVariance, 1e-9f),
                max(externalPosePositionVariance, 1e-9f);
            gtSAMgraph.add(PriorFactor<Pose3>(
                cloudKeyPoses3D->size(), externalPose,
                noiseModel::Diagonal::Variances(poseVariances)));
            aLoopIsClosed = true;
            RCLCPP_INFO_ONCE(
                get_logger(),
                "Using non-GPS external pose factors from %s",
                externalPoseTopic.c_str());
            break;
        }
    }

    void addLoopFactor() {
        if (loopIndexQueue.empty()) return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i) {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            auto noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor() {
        if (saveFrame() == false) return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // Optional simulator / wheel-odometry pose factor. This is kept
        // separate from the RTK / GPS path and is disabled by default.
        addExternalPoseFactor();

        // loop factor
        addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true) {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size();  // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity;  // this can be used as index
        thisPose6D.roll = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // save path for visualization
        updatePath(thisPose6D);

        int curcnt = cloudKeyPoses3D->size() - 1;
        if (loopClosureEnableFlag) {
            std::lock_guard<std::mutex> scanContextLock(mtxScanContext);
            // Scan Context loop detector - giseop
            // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected + downsampling)
            // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
            // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond region, so to solve this issue ... )
            const SCInputType sc_input_type = SCInputType::SINGLE_SCAN_FULL;  // change this

            if (sc_input_type == SCInputType::SINGLE_SCAN_FULL) {
                scManager.makeAndSaveScancontextAndKeys(*laserCloudRawDS);
            } else if (sc_input_type == SCInputType::SINGLE_SCAN_FEAT) {
                scManager.makeAndSaveScancontextAndKeys(*thisSurfKeyFrame);
            } else if (sc_input_type == SCInputType::MULTI_SCAN_FEAT) {
                pcl::PointCloud<PointType>::Ptr multiKeyFrameFeatureCloud(new pcl::PointCloud<PointType>());
                loopFindNearKeyframes(multiKeyFrameFeatureCloud, curcnt, historyKeyframeSearchNum);
                scManager.makeAndSaveScancontextAndKeys(*multiKeyFrameFeatureCloud);
            }

            // save sc data
            if (savePCD) {
                const auto& curr_scd = scManager.getConstRefRecentSCD();
                std::string curr_scd_node_idx = padZeros(curcnt);
                saveSCD(savePCDDirectory + "SCDs/" + curr_scd_node_idx + ".scd", curr_scd);
            }
        }

        if (savePCD) {
            savePCDIfNotEmpty(
                savePCDDirectory + "CornerMap/" + std::to_string(curcnt) + ".pcd",
                *thisCornerKeyFrame);
            savePCDIfNotEmpty(
                savePCDDirectory + "SurfMap/" + std::to_string(curcnt) + ".pcd",
                *thisSurfKeyFrame);
            savePCDIfNotEmpty(
                savePCDDirectory + "Scans/" + std::to_string(curcnt) + ".pcd",
                *laserCloudRawDS);
        }
    }

    void correctPoses() {
        if (cloudKeyPoses3D->points.empty()) return;

        if (aLoopIsClosed == true) {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i) {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = rclcpp::Time(pose_in.time * 1e9);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf2::Quaternion q;
        q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry() {
        // Publish odometry for ROS (global)
        nav_msgs::msg::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        laserOdometryROS.pose.pose.orientation = quat_msg;
        pubLaserOdometryGlobal->publish(laserOdometryROS);

        // Publish TF
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        tf2::Transform t_odom_to_lidar = tf2::Transform(quat_tf, tf2::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf2::TimePoint time_point = tf2_ros::fromRclcpp(timeLaserInfoStamp);
        tf2::Stamped<tf2::Transform> temp_odom_to_lidar(t_odom_to_lidar, time_point, odometryFrame);
        geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
        tf2::convert(temp_odom_to_lidar, trans_odom_to_lidar);
        trans_odom_to_lidar.child_frame_id = "lidar_link";
        if (publishMappingOdomTF) br->sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::msg::Odometry laserOdomIncremental;  // incremental odometry msg
        static Eigen::Affine3f increOdomAffine;               // incremental odometry in affine
        if (lastIncreOdomPubFlag == false) {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imu_available == true) {
                if (std::abs(cloudInfo.imu_pitch_init) < 1.4) {
                    double imuWeight = 0.1;
                    tf2::Quaternion imuQuaternion;
                    tf2::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imu_roll_init, 0, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imu_pitch_init, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);
            laserOdomIncremental.pose.pose.orientation = quat_msg;
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental->publish(laserOdomIncremental);
    }

    void publishFrames() {
        if (cloudKeyPoses3D->points.empty()) return;
        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame->get_subscription_count() != 0) {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        // if (pubCloudRegisteredRaw->get_subscription_count() != 0) {
        //     pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        //     pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
        //     PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        //     *cloudOut = *transformPointCloud(cloudOut, &thisPose6D);
        //     publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        // }
        // publish path
        if (pubPath->get_subscription_count() != 0) {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath->publish(globalPath);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto MO = std::make_shared<mapOptimization>(options);
    exec.add_node(MO);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Map Optimization Started.\033[0m");

    std::thread loopthread(&mapOptimization::loopClosureThread, MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, MO);

    exec.spin();

    rclcpp::shutdown();

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}
