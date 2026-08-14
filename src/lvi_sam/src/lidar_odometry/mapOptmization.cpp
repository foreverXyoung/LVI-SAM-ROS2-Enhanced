#include "utility.hpp"
#include "lvi_sam/internal_odom_metadata.hpp"
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
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <pcl/common/angles.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <opencv2/opencv.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <lvi_sam_msgs/msg/localization_status.hpp>
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

class MapOptimization : public ParamServer {
public:
    static constexpr const char* kMapWriteInProgressMarker =
        ".lvi_sam_mapping_in_progress";

    bool prepareFreshMapOutputDirectory() {
        namespace fs = std::filesystem;
        const fs::path root(savePCDDirectory);
        const std::set<std::string> allowedEmptyDirectories{
            "Scans", "SCDs", "SurfMap", "CornerMap"};
        std::error_code error;
        for (const auto& entry : fs::directory_iterator(root, error)) {
            if (error) break;
            const std::string name = entry.path().filename().string();
            if (entry.is_directory(error) && !error &&
                allowedEmptyDirectories.count(name) != 0 &&
                fs::is_empty(entry.path(), error) && !error) {
                continue;
            }
            RCLCPP_ERROR(
                get_logger(),
                "Map output directory is not fresh; found existing entry: %s",
                entry.path().string().c_str());
            return false;
        }
        if (error) {
            RCLCPP_ERROR(
                get_logger(), "Unable to inspect map output directory %s: %s",
                savePCDDirectory.c_str(), error.message().c_str());
            return false;
        }

        const std::string markerPath =
            savePCDDirectory + kMapWriteInProgressMarker;
        std::ofstream marker(markerPath, std::ios::trunc);
        marker << "Map serialization is incomplete until map_manifest.yaml "
                  "is committed.\n";
        marker.flush();
        if (!marker.good()) {
            RCLCPP_ERROR(
                get_logger(), "Unable to create map transaction marker: %s",
                markerPath.c_str());
            return false;
        }
        return true;
    }

    template <typename PointT>
    bool savePCDIfNotEmpty(const std::string& path,
                           const pcl::PointCloud<PointT>& cloud,
                           bool binary = true) {
        if (cloud.empty()) {
            RCLCPP_WARN(get_logger(),
                        "Skipping empty PCD output: %s", path.c_str());
            // PCL's binary writer rejects empty clouds. Keep an explicit
            // zero-byte marker so keyframe indices remain continuous; the
            // paired prior-map loader treats that marker as an empty feature
            // cloud and never passes it to loadPCDFile().
            std::ofstream emptyMarker(path, std::ios::out | std::ios::trunc);
            if (!emptyMarker.good()) {
                RCLCPP_ERROR(get_logger(),
                             "Failed to clear stale PCD output: %s", path.c_str());
            }
            return emptyMarker.good();
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
    std::unique_ptr<ISAM2> isam;
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
    std::deque<nav_msgs::msg::Odometry> localizationRtkQueue;
    double lastGpsInputTime = -std::numeric_limits<double>::infinity();
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
    rclcpp::Time loopSnapshotStamp{0, 0, RCL_ROS_TIME};
    double loopSnapshotTime = -1.0;

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
    int imuPreintegrationResetId = 0;
    bool mapArtifactWriteFailed = false;
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
    rclcpp::Publisher<lvi_sam_msgs::msg::LocalizationStatus>::SharedPtr
        pubLocalizationStatus;
    rclcpp::TimerBase::SharedPtr localizationStatusTimer;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srvForceRelocalize;

    /////////////////////////////////// SC Start ///////////////////////////////////
    SCManager scManager;
    pcl::PointCloud<PointType>::Ptr laserCloudRaw{new pcl::PointCloud<PointType>()};    // giseop
    pcl::PointCloud<PointType>::Ptr laserCloudRawDS{new pcl::PointCloud<PointType>()};  // giseop
    /////////////////////////////////// SC End ///////////////////////////////////

    explicit MapOptimization(const rclcpp::NodeOptions& options)
        : ParamServer("mapOptimizationParamServer", options),
          featureExtraction(makeInternalNodeOptions(
              options, "lvi_sam_feature_extraction_internal")),
          imageProjection(makeInternalNodeOptions(
              options, "lvi_sam_image_projection_internal")) {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = std::make_unique<ISAM2>(parameters);

        pubKeyPoses = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/trajectory", 1);
        pubLaserCloudSurround = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>("lio_sam/mapping/odometry", qos);
        pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>("lio_sam/mapping/odometry_incremental", qos);
        pubPath = create_publisher<nav_msgs::msg::Path>("lio_sam/mapping/path", 1);
        br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        subImu = create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, qos_imu,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    hasImuInput = true;
                }
                imageProjection.imuHandler(msg);
            });
        subOdom = create_subscription<nav_msgs::msg::Odometry>(
            odomTopic + "_incremental", qos_imu,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    hasIncrementalOdomInput = true;
                }
                imageProjection.odometryHandler(msg);
            });

        subCloud = create_subscription<livox_ros_driver2::msg::CustomMsg>(pointCloudTopic, qos_lidar,
                                                                          std::bind(&MapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
        if (useExternalPoseFactor) {
            subExternalPose = create_subscription<nav_msgs::msg::Odometry>(
                externalPoseTopic, 200,
                std::bind(&MapOptimization::externalPoseHandler, this,
                          std::placeholders::_1));
        }
        subLoop = create_subscription<std_msgs::msg::Float64MultiArray>("lio_loop/loop_closure_detection", qos,
                                                                        std::bind(&MapOptimization::loopInfoHandler, this, std::placeholders::_1));

        // srvSaveMap = create_service<lio_sam::srv::SaveMap>("lio_sam/save_map", saveMapService);
        pubHistoryKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
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
        rclcpp::QoS localizationStatusQos(1);
        localizationStatusQos.reliable().transient_local();
        pubLocalizationStatus = create_publisher<lvi_sam_msgs::msg::LocalizationStatus>(
            "lio_sam/localization/status", localizationStatusQos);

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
            if (!prepareFreshMapOutputDirectory()) {
                throw std::runtime_error(
                    "savePCDDirectory must be a new or empty directory: " +
                    savePCDDirectory);
            }
        }

        InitLocationMode();
        localizationStatusTimer = create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() {
                std::lock_guard<std::mutex> lock(mtx);
                publishStructuredLocalizationStatus();
            });
        publishStructuredLocalizationStatus(true);

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
                std::bind(&MapOptimization::gpsHandler, this,
                          std::placeholders::_1));
        }

        srvForceRelocalize = create_service<std_srvs::srv::Trigger>(
            "/lio_sam/localization/force_relocalize",
            std::bind(
                &MapOptimization::forceRelocalizeHandler,
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
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i) {
            transformTobeMapped[i] = 0;
        }

    }

    /////////////////////////////////// Loc Start ///////////////////////////////////

    bool LocEnableFlag = false;
    std::string loadPCDDirectory;
    PointType lastPoses3D;
    PointTypePose lastPoses6D;
    int surroundingKeyframeSearchMaxNum = 10;

    // ReLoc
    bool subMapMode = true;
    bool useSCReLoc = false;
    bool useRTKAssist = false;
    bool useRTKInitialization = false;
    bool rtkUseHeading = false;
    bool rtkAdaptiveBlend = true;
    float rtkPositionBlend = 0.20f;
    float rtkMaxInnovation = 3.0f;
    float rtkMaxNormalizedInnovation = 5.0f;
    float rtkYawVarianceThreshold = 0.10f;
    float rtkInitializationMaxSpread = 0.50f;
    int rtkInitializationMinSamples = 3;
    std::string rtkExpectedFrame;
    std::deque<nav_msgs::msg::Odometry> rtkInitializationSamples;
    bool dynamicFilterEnable = false;
    bool dynamicFilterPublishDebug = true;
    float dynamicFilterMaxMapDistance = 0.80f;
    float dynamicFilterInitialMaxMapDistance = 1.50f;
    int dynamicFilterWarmupFrames = 10;
    float dynamicFilterMinKeepRatio = 0.25f;
    int dynamicFilterMinCornerPoints = 11;
    int dynamicFilterMinSurfPoints = 101;
    int dynamicFilterFrameCount = 0;
    bool lostDetectionEnable = true;
    int lostBadMatchThreshold = 5;
    int localizationBadMatchCount = 0;
    bool resetInitialGuessSeed = false;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDSFiltered;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDSFiltered;
    pcl::PointCloud<PointType>::Ptr dynamicFilterKeptPoints;
    pcl::PointCloud<PointType>::Ptr dynamicFilterRejectedPoints;
    std::vector<double> init_guess;
    pcl::PointCloud<PointType>::Ptr idxMap{new pcl::PointCloud<PointType>};
    enum InitializedFlag { NonInitialized, Initializing, Initialized, MayLost };
    InitializedFlag LocInitSta = InitializedFlag::NonInitialized;
    int relocalizationAttemptCount = 0;
    std::string lastPublishedLocalizationState;
    bool priorMapReady = false;
    bool hasValidLidarInput = false;
    bool hasImuInput = false;
    bool hasIncrementalOdomInput = false;
    bool hasProcessedLidar = false;
    bool localizationLossEventPending = false;
    uint32_t localizationConsecutiveSuccesses = 0;
    uint8_t lastStructuredLocalizationState = 255;
    uint8_t previousStructuredLocalizationState = 255;
    uint64_t localizationStatusTransitionSequence = 0;
    uint32_t localizationLossCount = 0;
    rclcpp::Time localizationStateEnteredAt{0, 0, RCL_ROS_TIME};
    rclcpp::Time lastValidLocalizationPoseAt{0, 0, RCL_ROS_TIME};
    rclcpp::Time lastLocalizationLossAt{0, 0, RCL_ROS_TIME};

    template <typename T>
    void declare_and_get_parameter(const std::string& name, T& variable, const T& default_value) {
        this->declare_parameter<T>(name, default_value);
        this->get_parameter(name, variable);
    }

    std::string localizationStateName() const {
        if (!LocEnableFlag) return "MAPPING";
        switch (LocInitSta) {
            case InitializedFlag::NonInitialized:
                return "RELOCALIZING";
            case InitializedFlag::Initializing:
                return "RELOCALIZING";
            case InitializedFlag::Initialized:
                return "LOCALIZED";
            case InitializedFlag::MayLost:
                return "LOST";
            default:
                return "UNKNOWN";
        }
    }

    uint8_t structuredLocalizationState() const {
        if (!LocEnableFlag) return lvi_sam_msgs::msg::LocalizationStatus::MAPPING;
        switch (LocInitSta) {
            case InitializedFlag::NonInitialized:
            case InitializedFlag::Initializing:
                return lvi_sam_msgs::msg::LocalizationStatus::RELOCALIZING;
            case InitializedFlag::Initialized:
                return lvi_sam_msgs::msg::LocalizationStatus::TRACKING;
            case InitializedFlag::MayLost:
                return lvi_sam_msgs::msg::LocalizationStatus::LOST;
            default:
                return lvi_sam_msgs::msg::LocalizationStatus::STATE_UNKNOWN;
        }
    }

    static std::string structuredLocalizationStateName(const uint8_t state) {
        switch (state) {
            case lvi_sam_msgs::msg::LocalizationStatus::MAPPING:
                return "MAPPING";
            case lvi_sam_msgs::msg::LocalizationStatus::RELOCALIZING:
                return "RELOCALIZING";
            case lvi_sam_msgs::msg::LocalizationStatus::TRACKING:
                return "TRACKING";
            case lvi_sam_msgs::msg::LocalizationStatus::LOST:
                return "LOST";
            case lvi_sam_msgs::msg::LocalizationStatus::VERIFYING:
                return "VERIFYING";
            case lvi_sam_msgs::msg::LocalizationStatus::DEGRADED:
                return "DEGRADED";
            case lvi_sam_msgs::msg::LocalizationStatus::WAITING_FOR_SENSORS:
                return "WAITING_FOR_SENSORS";
            case lvi_sam_msgs::msg::LocalizationStatus::ERROR:
                return "ERROR";
            default:
                return "UNKNOWN";
        }
    }

    void publishStructuredLocalizationStatus(bool force = false) {
        if (!pubLocalizationStatus) return;

        const auto now = this->now();
        // MayLost is intentionally transient in the frozen algorithm: the
        // same LiDAR callback immediately returns to NonInitialized. Latch it
        // only for this status stream so consumers cannot miss the loss event.
        const bool reportingLatchedLoss = localizationLossEventPending;
        const uint8_t state = reportingLatchedLoss
            ? lvi_sam_msgs::msg::LocalizationStatus::LOST
            : structuredLocalizationState();
        if (reportingLatchedLoss) localizationLossEventPending = false;
        const bool stateChanged = state != lastStructuredLocalizationState;
        if (stateChanged) {
            previousStructuredLocalizationState = lastStructuredLocalizationState;
            lastStructuredLocalizationState = state;
            ++localizationStatusTransitionSequence;
            localizationStateEnteredAt = now;
            if (state == lvi_sam_msgs::msg::LocalizationStatus::TRACKING) {
                lastValidLocalizationPoseAt = now;
            }
            if (state == lvi_sam_msgs::msg::LocalizationStatus::LOST) {
                lastLocalizationLossAt = now;
                ++localizationLossCount;
            }
        }

        // `force` is intentionally only a publication hint. It never changes
        // the underlying algorithm state; the timer publishes the same
        // snapshot periodically as a low-rate heartbeat.
        (void)force;
        lvi_sam_msgs::msg::LocalizationStatus msg;
        msg.header.stamp = now.to_msg();
        msg.state = state;
        msg.previous_state = previousStructuredLocalizationState;
        msg.mode = LocEnableFlag
            ? lvi_sam_msgs::msg::LocalizationStatus::MODE_LOCALIZATION
            : lvi_sam_msgs::msg::LocalizationStatus::MODE_MAPPING;
        msg.state_name = structuredLocalizationStateName(state);
        msg.previous_state_name =
            structuredLocalizationStateName(previousStructuredLocalizationState);
        switch (state) {
            case lvi_sam_msgs::msg::LocalizationStatus::MAPPING:
                msg.reason = "mapping_mode";
                break;
            case lvi_sam_msgs::msg::LocalizationStatus::RELOCALIZING:
                msg.reason = "relocalization_pending";
                break;
            case lvi_sam_msgs::msg::LocalizationStatus::TRACKING:
                msg.reason = "legacy_localization_initialized";
                break;
            case lvi_sam_msgs::msg::LocalizationStatus::LOST:
                msg.reason = "legacy_bad_match_threshold";
                break;
            default:
                msg.reason = "state_not_emitted_in_phase_1";
                break;
        }

        const bool poseValid =
            LocEnableFlag && LocInitSta == InitializedFlag::Initialized;
        msg.pose_valid = poseValid;
        msg.odometry_valid = hasProcessedLidar && (!LocEnableFlag || poseValid);
        msg.sensors_ready =
            hasValidLidarInput && hasImuInput && hasIncrementalOdomInput;
        msg.map_ready = priorMapReady;
        msg.relocalization_active =
            state == lvi_sam_msgs::msg::LocalizationStatus::RELOCALIZING ||
            state == lvi_sam_msgs::msg::LocalizationStatus::LOST;
        msg.quality_degraded = false;

        // Quality metrics are reserved for a later phase. -1 is the explicit
        // unknown sentinel and prevents consumers from mistaking a placeholder
        // for a real matcher score or timing measurement.
        msg.match_score = -1.0f;
        msg.confidence = -1.0f;
        msg.lidar_age = -1.0f;
        msg.imu_age = -1.0f;
        msg.odometry_age = -1.0f;
        msg.consecutive_successes = localizationConsecutiveSuccesses;
        msg.consecutive_failures =
            static_cast<uint32_t>(std::max(0, localizationBadMatchCount));
        msg.relocalization_attempts =
            static_cast<uint32_t>(std::max(0, relocalizationAttemptCount));
        msg.transition_sequence = localizationStatusTransitionSequence;
        msg.loss_count = localizationLossCount;
        msg.last_valid_pose_stamp = lastValidLocalizationPoseAt.to_msg();
        msg.state_enter_stamp = localizationStateEnteredAt.to_msg();
        msg.last_lost_stamp = lastLocalizationLossAt.to_msg();
        pubLocalizationStatus->publish(msg);
    }

    void publishLocalizationState(bool force = false) {
        const std::string state = localizationStateName();
        if (pubLocalizationState) {
            static double lastPublishTime = -1.0;
            const double now = this->now().seconds();
            if (force || state != lastPublishedLocalizationState ||
                now - lastPublishTime >= 1.0) {
                std_msgs::msg::String msg;
                msg.data = state;
                pubLocalizationState->publish(msg);
                lastPublishedLocalizationState = state;
                lastPublishTime = now;
            }
        }
    }

    void forceRelocalizeHandler(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        std::lock_guard<std::mutex> lock(mtx);

        if (!LocEnableFlag) {
            response->success = false;
            response->message =
                "LIO-SAM is running in mapping mode; forced relocalization is unavailable.";
            return;
        }

        LocInitSta = InitializedFlag::NonInitialized;
        localizationBadMatchCount = 0;
        localizationConsecutiveSuccesses = 0;
        localizationLossEventPending = false;
        dynamicFilterFrameCount = 0;
        relocalizationAttemptCount = 0;
        rtkInitializationSamples.clear();
        resetInitialGuessSeed = true;
        publishLocalizationState(true);

        response->success = true;
        response->message =
            "Forced relocalization scheduled; the next processed LiDAR frame will run the configured relocalization path.";
        RCLCPP_WARN(
            get_logger(),
            "Forced relocalization requested. Pause Nav2 motion until localization returns to LOCALIZED.");
    }

    void InitLocationMode() {
        declare_and_get_parameter<bool>("Loc.EnableFlag", LocEnableFlag, false);
        if (LocEnableFlag) {
            declare_and_get_parameter<bool>("Loc.useSCReLoc", useSCReLoc, false);
            declare_and_get_parameter<bool>("Loc.useRTKAssist", useRTKAssist, false);
            declare_and_get_parameter<bool>("Loc.useRTKInitialization", useRTKInitialization, false);
            declare_and_get_parameter<bool>("Loc.rtkUseHeading", rtkUseHeading, false);
            declare_and_get_parameter<bool>("Loc.rtkAdaptiveBlend", rtkAdaptiveBlend, true);
            declare_and_get_parameter<float>("Loc.rtkPositionBlend", rtkPositionBlend, 0.20f);
            declare_and_get_parameter<float>("Loc.rtkMaxInnovation", rtkMaxInnovation, 3.0f);
            declare_and_get_parameter<float>("Loc.rtkMaxNormalizedInnovation", rtkMaxNormalizedInnovation, 5.0f);
            declare_and_get_parameter<float>("Loc.rtkYawVarianceThreshold", rtkYawVarianceThreshold, 0.10f);
            declare_and_get_parameter<float>("Loc.rtkInitializationMaxSpread", rtkInitializationMaxSpread, 0.50f);
            declare_and_get_parameter<int>("Loc.rtkInitializationMinSamples", rtkInitializationMinSamples, 3);
            declare_and_get_parameter<std::string>("Loc.rtkExpectedFrame", rtkExpectedFrame, "");
            declare_and_get_parameter<bool>("Loc.dynamicFilterEnable", dynamicFilterEnable, false);
            declare_and_get_parameter<bool>("Loc.dynamicFilterPublishDebug", dynamicFilterPublishDebug, true);
            declare_and_get_parameter<float>("Loc.dynamicFilterMaxMapDistance", dynamicFilterMaxMapDistance, 0.80f);
            declare_and_get_parameter<float>("Loc.dynamicFilterInitialMaxMapDistance", dynamicFilterInitialMaxMapDistance, 1.50f);
            declare_and_get_parameter<int>("Loc.dynamicFilterWarmupFrames", dynamicFilterWarmupFrames, 10);
            declare_and_get_parameter<float>("Loc.dynamicFilterMinKeepRatio", dynamicFilterMinKeepRatio, 0.25f);
            declare_and_get_parameter<int>("Loc.dynamicFilterMinCornerPoints", dynamicFilterMinCornerPoints, 11);
            declare_and_get_parameter<int>("Loc.dynamicFilterMinSurfPoints", dynamicFilterMinSurfPoints, 101);
            declare_and_get_parameter<bool>("Loc.lostDetectionEnable", lostDetectionEnable, true);
            declare_and_get_parameter<int>("Loc.lostBadMatchThreshold", lostBadMatchThreshold, 5);
            declare_and_get_parameter<vector<double>>("Loc.init_guess", init_guess, vector<double>());
            declare_and_get_parameter<string>("Loc.loadPCDDirectory", loadPCDDirectory, "");
            if (!useSCReLoc && init_guess.size() != 6) {
                throw std::runtime_error(
                    "Loc.init_guess must contain [roll,pitch,yaw,x,y,z] when Scan Context relocalization is disabled");
            }
            declare_and_get_parameter<float>("Loc.surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 20.0);
            declare_and_get_parameter<int>("Loc.surroundingKeyframeSearchMaxNum", surroundingKeyframeSearchMaxNum, 10);
            if (!std::all_of(init_guess.begin(), init_guess.end(),
                             [](const double value) {
                                 return std::isfinite(value);
                             })) {
                throw std::runtime_error(
                    "Loc.init_guess must contain only finite values");
            }
            if (!std::isfinite(rtkPositionBlend) || rtkPositionBlend < 0.0f ||
                rtkPositionBlend > 1.0f || !std::isfinite(rtkMaxInnovation) ||
                rtkMaxInnovation <= 0.0f ||
                !std::isfinite(rtkMaxNormalizedInnovation) ||
                rtkMaxNormalizedInnovation <= 0.0f ||
                !std::isfinite(rtkYawVarianceThreshold) ||
                rtkYawVarianceThreshold <= 0.0f ||
                !std::isfinite(rtkInitializationMaxSpread) ||
                rtkInitializationMaxSpread < 0.0f ||
                rtkInitializationMinSamples <= 0) {
                throw std::runtime_error(
                    "Loc RTK blend/innovation/covariance/initialization parameters are invalid");
            }
            if (useRTKInitialization && !useRTKAssist) {
                throw std::runtime_error(
                    "Loc.useRTKInitialization requires Loc.useRTKAssist=true");
            }
            if (!std::isfinite(dynamicFilterMaxMapDistance) ||
                !std::isfinite(dynamicFilterInitialMaxMapDistance) ||
                !std::isfinite(dynamicFilterMinKeepRatio) ||
                dynamicFilterMaxMapDistance <= 0.0f ||
                dynamicFilterInitialMaxMapDistance <= 0.0f ||
                dynamicFilterWarmupFrames < 0 ||
                dynamicFilterMinKeepRatio <= 0.0f ||
                dynamicFilterMinKeepRatio > 1.0f ||
                dynamicFilterMinCornerPoints <= 0 ||
                dynamicFilterMinSurfPoints <= 0 || lostBadMatchThreshold <= 0) {
                throw std::runtime_error(
                    "Loc dynamic-filter and loss-detection parameters are invalid");
            }
            if (loadPCDDirectory.empty() ||
                !std::isfinite(surroundingKeyframeSearchRadius) ||
                surroundingKeyframeSearchRadius <= 0.0f ||
                surroundingKeyframeSearchMaxNum <= 0) {
                throw std::runtime_error(
                    "Loc prior-map directory/search parameters are invalid");
            }
            LoadPriorMap();
            priorMapReady = true;
            publishLocalizationState(true);
        }
    }

    bool validateMapManifest(const std::size_t loadedKeyframeCount) {
        const std::string manifestPath = loadPCDDirectory + "/map_manifest.yaml";
        std::ifstream manifest(manifestPath);
        if (!manifest.is_open()) {
            RCLCPP_WARN(
                get_logger(),
                "Prior map has no map_manifest.yaml; loading it as a legacy map");
            return false;
        }

        std::map<std::string, std::string> values;
        const auto trim = [](std::string value) {
            const std::size_t first = value.find_first_not_of(" \t\r");
            if (first == std::string::npos) return std::string{};
            const std::size_t last = value.find_last_not_of(" \t\r");
            return value.substr(first, last - first + 1);
        };
        std::string line;
        while (std::getline(manifest, line)) {
            const std::size_t separator = line.find(':');
            if (separator == std::string::npos) continue;
            const std::string key = trim(line.substr(0, separator));
            const std::string value = trim(line.substr(separator + 1));
            if (!key.empty()) values[key] = value;
        }

        const auto parseInteger = [&](const char* key) {
            const auto found = values.find(key);
            if (found == values.end()) {
                throw std::runtime_error(
                    "Prior map manifest is missing " + std::string(key) +
                    ": " + manifestPath);
            }
            try {
                std::size_t parsedCharacters = 0;
                const long long result =
                    std::stoll(found->second, &parsedCharacters);
                if (parsedCharacters != found->second.size())
                    throw std::invalid_argument("trailing characters");
                return result;
            } catch (const std::exception&) {
                throw std::runtime_error(
                    "Prior map manifest contains an invalid " +
                    std::string(key) + ": " + manifestPath);
            }
        };

        const long long schemaVersion = parseInteger("schema_version");
        if (schemaVersion < 1 || schemaVersion > 1) {
            throw std::runtime_error(
                "Unsupported prior map schema version: " +
                std::to_string(schemaVersion));
        }
        const long long declaredKeyframeCount = parseInteger("keyframe_count");
        if (declaredKeyframeCount < 0 ||
            static_cast<std::size_t>(declaredKeyframeCount) !=
                loadedKeyframeCount) {
            throw std::runtime_error(
                "Prior map manifest keyframe_count does not match PCD data");
        }
        if (parseInteger("scan_context_rings") != scManager.PC_NUM_RING) {
            throw std::runtime_error(
                "Prior map Scan Context ring count is incompatible");
        }
        if (parseInteger("scan_context_sectors") != scManager.PC_NUM_SECTOR) {
            throw std::runtime_error(
                "Prior map Scan Context sector count is incompatible");
        }
        const auto requireMatchingFrame = [&](const char* key,
                                              const std::string& configured) {
            const auto found = values.find(key);
            if (found == values.end()) {
                throw std::runtime_error(
                    "Prior map manifest is missing " + std::string(key) +
                    ": " + manifestPath);
            }
            if (found->second != configured) {
                throw std::runtime_error(
                    "Prior map " + std::string(key) + "=" + found->second +
                    " is incompatible with configured frame=" + configured);
            }
        };
        requireMatchingFrame("map_frame", mapFrame);
        requireMatchingFrame("odometry_frame", odometryFrame);
        requireMatchingFrame("lidar_frame", lidarFrame);
        return true;
    }

    bool writeMapManifest() const {
        const std::string manifestPath = savePCDDirectory + "map_manifest.yaml";
        const std::string temporaryPath = manifestPath + ".tmp";
        std::ofstream manifest(temporaryPath, std::ios::trunc);
        if (!manifest.is_open()) {
            RCLCPP_ERROR(
                get_logger(), "Unable to write map manifest: %s",
                temporaryPath.c_str());
            return false;
        }
        manifest << "schema_version: 1\n";
        manifest << "producer: lvi_sam_ros2_enhanced\n";
        manifest << "keyframe_count: " << cloudKeyPoses6D->size() << "\n";
        manifest << "map_frame: " << mapFrame << "\n";
        manifest << "odometry_frame: " << odometryFrame << "\n";
        manifest << "lidar_frame: " << lidarFrame << "\n";
        manifest << "scan_context_rings: " << scManager.PC_NUM_RING << "\n";
        manifest << "scan_context_sectors: " << scManager.PC_NUM_SECTOR << "\n";
        manifest << "scan_context_distance_threshold: "
                 << scManager.SC_DIST_THRES << "\n";
        manifest << "visual_database_optional: VisualMap/visual_keyframes.yaml\n";
        manifest.flush();
        if (!manifest.good()) {
            RCLCPP_ERROR(
                get_logger(), "Unable to finish map manifest: %s",
                temporaryPath.c_str());
            manifest.close();
            std::remove(temporaryPath.c_str());
            return false;
        }
        manifest.close();
        if (std::rename(temporaryPath.c_str(), manifestPath.c_str()) != 0) {
            RCLCPP_ERROR(
                get_logger(), "Unable to publish completed map manifest: %s",
                manifestPath.c_str());
            std::remove(temporaryPath.c_str());
            return false;
        }
        const std::string markerPath =
            savePCDDirectory + kMapWriteInProgressMarker;
        if (std::remove(markerPath.c_str()) != 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Map manifest was committed but the incomplete marker could "
                "not be removed: %s",
                markerPath.c_str());
            return false;
        }
        return true;
    }

    void LoadPriorMap() {
        if (loadPCDDirectory.empty()) {
            throw std::runtime_error("Loc.loadPCDDirectory must be set in localization mode");
        }
        if (loadPCDDirectory.front() != '/') {
            throw std::runtime_error(
                "Loc.loadPCDDirectory must be an absolute path: " +
                loadPCDDirectory);
        }
        const std::string incompleteMarker =
            loadPCDDirectory + "/" + kMapWriteInProgressMarker;
        if (std::filesystem::exists(incompleteMarker)) {
            throw std::runtime_error(
                "Prior map was not serialized completely: " +
                incompleteMarker);
        }
        cloudKeyPoses6D->clear();
        cloudKeyPoses3D->clear();
        const int transformationsResult = pcl::io::loadPCDFile<PointTypePose>(
            loadPCDDirectory + "/transformations.pcd", *cloudKeyPoses6D);
        const int trajectoryResult = pcl::io::loadPCDFile<PointType>(
            loadPCDDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
        if (transformationsResult < 0 || trajectoryResult < 0 ||
            cloudKeyPoses6D->empty() || cloudKeyPoses3D->empty()) {
            throw std::runtime_error(
                "Prior map is missing trajectory.pcd or transformations.pcd: " +
                loadPCDDirectory);
        }
        if (cloudKeyPoses6D->size() != cloudKeyPoses3D->size()) {
            throw std::runtime_error(
                "Prior map pose files have different keyframe counts: transformations=" +
                std::to_string(cloudKeyPoses6D->size()) + " trajectory=" +
                std::to_string(cloudKeyPoses3D->size()));
        }
        const bool hasValidatedManifest =
            validateMapManifest(cloudKeyPoses6D->size());
        for (std::size_t i = 0; i < cloudKeyPoses6D->size(); ++i) {
            auto& pose = cloudKeyPoses6D->points[i];
            auto& trajectoryPoint = cloudKeyPoses3D->points[i];
            if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
                !std::isfinite(pose.z) || !std::isfinite(pose.roll) ||
                !std::isfinite(pose.pitch) || !std::isfinite(pose.yaw) ||
                !std::isfinite(pose.time) ||
                !std::isfinite(trajectoryPoint.x) ||
                !std::isfinite(trajectoryPoint.y) ||
                !std::isfinite(trajectoryPoint.z)) {
                throw std::runtime_error(
                    "Prior map contains a non-finite keyframe pose at index " +
                    std::to_string(i));
            }
            // Runtime containers address keyframes by vector index. Normalize
            // legacy intensity fields once at the map boundary instead of
            // relying on possibly stale serialized IDs throughout the graph.
            pose.intensity = static_cast<float>(i);
            trajectoryPoint.intensity = static_cast<float>(i);
        }
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);

        int count = cloudKeyPoses6D->size();

        std::cout << "loading map [ " << count << " ] from " << loadPCDDirectory << std::endl;
        std::size_t loadedFeaturePointCount = 0;
        for (int i = 0; i < count; i++) {
            CloudPtr cornerKeyFrame(new CloudType());
            CloudPtr surfKeyFrame(new CloudType());
            const std::string cornerPath =
                loadPCDDirectory + "/CornerMap/" + std::to_string(i) + ".pcd";
            const std::string surfPath =
                loadPCDDirectory + "/SurfMap/" + std::to_string(i) + ".pcd";
            std::ifstream cornerFile(cornerPath,
                                     std::ios::in | std::ios::binary | std::ios::ate);
            std::ifstream surfFile(surfPath,
                                   std::ios::in | std::ios::binary | std::ios::ate);
            if (!cornerFile.good()) {
                if (hasValidatedManifest) {
                    throw std::runtime_error(
                        "Manifest-backed map is missing corner keyframe: " +
                        cornerPath);
                }
                RCLCPP_WARN(get_logger(), "Corner keyframe is missing: %s",
                            cornerPath.c_str());
            } else if (cornerFile.tellg() > std::streampos(0)) {
                if (pcl::io::loadPCDFile<PointType>(
                        cornerPath, *cornerKeyFrame) < 0) {
                    throw std::runtime_error(
                        "Unable to parse corner keyframe: " + cornerPath);
                }
                std::vector<int> validIndices;
                pcl::removeNaNFromPointCloud(
                    *cornerKeyFrame, *cornerKeyFrame, validIndices);
            } else {
                RCLCPP_WARN(get_logger(), "Corner keyframe is empty: %s",
                            cornerPath.c_str());
            }
            if (!surfFile.good()) {
                if (hasValidatedManifest) {
                    throw std::runtime_error(
                        "Manifest-backed map is missing surface keyframe: " +
                        surfPath);
                }
                RCLCPP_WARN(get_logger(), "Surface keyframe is missing: %s",
                            surfPath.c_str());
            } else if (surfFile.tellg() > std::streampos(0)) {
                if (pcl::io::loadPCDFile<PointType>(surfPath, *surfKeyFrame) <
                    0) {
                    throw std::runtime_error(
                        "Unable to parse surface keyframe: " + surfPath);
                }
                std::vector<int> validIndices;
                pcl::removeNaNFromPointCloud(
                    *surfKeyFrame, *surfKeyFrame, validIndices);
            } else {
                RCLCPP_WARN(get_logger(), "Surface keyframe is empty: %s",
                            surfPath.c_str());
            }
            loadedFeaturePointCount += cornerKeyFrame->size() + surfKeyFrame->size();
            cornerCloudKeyFrames.push_back(cornerKeyFrame);
            surfCloudKeyFrames.push_back(surfKeyFrame);
        }
        if (loadedFeaturePointCount == 0) {
            throw std::runtime_error("Prior map contains no usable CornerMap/SurfMap points");
        }
        std::cout << "************************Keyframe map loaded************************" << std::endl;

        if (useSCReLoc == false) return;  // 提前结束

        // 如果不存在idxMap，即非submap模式
        if (pcl::io::loadPCDFile<PointType>(loadPCDDirectory + "/idxMap.pcd", *idxMap) == -1) {
            if (pcl::io::loadPCDFile<PointType>(loadPCDDirectory + "/trajectory.pcd", *idxMap) < 0) {
                throw std::runtime_error("Unable to construct Scan Context index map");
            }
            subMapMode = false;
        }
        if (idxMap->empty()) {
            throw std::runtime_error("Scan Context index map is empty");
        }
        int scRows = -1;
        int scCols = -1;
        for (std::size_t i = 0; i < idxMap->size(); ++i) {
            std::string scd_path = loadPCDDirectory + "/SCDs/" + padZeros(i) + ".scd";
            Eigen::MatrixXd load_sc;
            if (!loadSCD(scd_path, load_sc)) {
                throw std::runtime_error("Invalid or missing Scan Context descriptor: " + scd_path);
            }
            if (load_sc.rows() != scManager.PC_NUM_RING ||
                load_sc.cols() != scManager.PC_NUM_SECTOR) {
                throw std::runtime_error("Unexpected Scan Context descriptor dimensions: " + scd_path);
            }
            if (scRows < 0) {
                scRows = load_sc.rows();
                scCols = load_sc.cols();
            } else if (load_sc.rows() != scRows || load_sc.cols() != scCols) {
                throw std::runtime_error("Inconsistent Scan Context descriptor dimensions: " + scd_path);
            }

            // load keys
            Eigen::MatrixXd ringkey = scManager.makeRingkeyFromScancontext(load_sc);
            Eigen::MatrixXd sectorkey = scManager.makeSectorkeyFromScancontext(load_sc);
            std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);

            scManager.polarcontexts_.push_back(load_sc);
            scManager.polarcontext_invkeys_.push_back(ringkey);
            scManager.polarcontext_vkeys_.push_back(sectorkey);
            scManager.polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec);
        }

        std::cout << "************************sc loaded************************" << std::endl;
    }

    void saveKeyFramesAndLoc() {
        if (saveFrame() == false) return;

        lastPoses6D = trans2PointTypePose(transformTobeMapped);
        lastPoses3D.x = lastPoses6D.x, lastPoses3D.y = lastPoses6D.y, lastPoses3D.z = lastPoses6D.z;
        // lastPoses3D.intensity = lastPoses6D.intensity =  i;
        lastPoses6D.time = timeLaserInfoCur;

        updatePath(lastPoses6D);
    }

    bool validateMapAlignedRTK(
        const nav_msgs::msg::Odometry& rtkOdom,
        bool requireHeading,
        std::string* rejectionReason = nullptr) const {
        auto reject = [&](const std::string& reason) {
            if (rejectionReason != nullptr) *rejectionReason = reason;
            return false;
        };

        if (!std::isfinite(stamp2Sec(rtkOdom.header.stamp))) {
            return reject("non-finite timestamp");
        }

        const std::string& expectedFrame =
            LocEnableFlag && !rtkExpectedFrame.empty()
                ? rtkExpectedFrame
                : gpsExpectedFrame;
        if (!expectedFrame.empty() &&
            rtkOdom.header.frame_id != expectedFrame) {
            return reject("unexpected frame_id=" + rtkOdom.header.frame_id);
        }

        const double x = rtkOdom.pose.pose.position.x;
        const double y = rtkOdom.pose.pose.position.y;
        const double varianceX = rtkOdom.pose.covariance[0];
        const double varianceY = rtkOdom.pose.covariance[7];
        if (!std::isfinite(x) || !std::isfinite(y)) {
            return reject("non-finite position");
        }
        if (!std::isfinite(varianceX) || !std::isfinite(varianceY) ||
            varianceX <= 0.0 || varianceY <= 0.0) {
            return reject("missing or invalid XY covariance");
        }
        if (varianceX > gpsCovThreshold || varianceY > gpsCovThreshold) {
            return reject("XY covariance exceeds gpsCovThreshold");
        }

        if (requireHeading) {
            const auto& q = rtkOdom.pose.pose.orientation;
            const double quaternionNorm = std::sqrt(
                q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            const double yawVariance = rtkOdom.pose.covariance[35];
            if (!std::isfinite(quaternionNorm) || quaternionNorm < 0.99 ||
                quaternionNorm > 1.01) {
                return reject("invalid heading quaternion");
            }
            if (!std::isfinite(yawVariance) || yawVariance <= 0.0 ||
                yawVariance > rtkYawVarianceThreshold) {
                return reject("yaw covariance exceeds RTK heading threshold");
            }
        }
        return true;
    }

    bool getLocalizationRTK(nav_msgs::msg::Odometry& rtkOdom) {
        std::lock_guard<std::mutex> lock(mtxGPS);
        while (!localizationRtkQueue.empty()) {
            const double gpsTime =
                stamp2Sec(localizationRtkQueue.front().header.stamp);
            if (gpsTime < timeLaserInfoCur - gpsTimeTolerance) {
                localizationRtkQueue.pop_front();
                continue;
            }
            if (gpsTime > timeLaserInfoCur + gpsTimeTolerance) return false;

            rtkOdom = localizationRtkQueue.front();
            localizationRtkQueue.pop_front();
            std::string reason;
            if (!validateMapAlignedRTK(rtkOdom, rtkUseHeading, &reason)) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Rejecting RTK localization sample: %s", reason.c_str());
                continue;
            }
            return true;
        }
        return false;
    }

    bool initializeLocalizationFromRTK() {
        if (!useRTKAssist || !useRTKInitialization) return false;
        nav_msgs::msg::Odometry rtkOdom;
        if (!getLocalizationRTK(rtkOdom)) return false;

        rtkInitializationSamples.push_back(rtkOdom);
        while (rtkInitializationSamples.size() >
               static_cast<std::size_t>(rtkInitializationMinSamples)) {
            rtkInitializationSamples.pop_front();
        }
        if (rtkInitializationSamples.size() <
            static_cast<std::size_t>(rtkInitializationMinSamples)) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for stable RTK initialization samples: %zu/%d",
                rtkInitializationSamples.size(), rtkInitializationMinSamples);
            return false;
        }

        double weightSum = 0.0;
        double xSum = 0.0;
        double ySum = 0.0;
        double yawSinSum = 0.0;
        double yawCosSum = 0.0;
        for (const auto& sample : rtkInitializationSamples) {
            const double variance = std::max<double>(
                gpsVarianceFloor,
                std::max<double>(sample.pose.covariance[0],
                                 sample.pose.covariance[7]));
            const double weight = 1.0 / variance;
            weightSum += weight;
            xSum += weight * sample.pose.pose.position.x;
            ySum += weight * sample.pose.pose.position.y;
            if (rtkUseHeading) {
                tf2::Quaternion quaternion;
                tf2::fromMsg(sample.pose.pose.orientation, quaternion);
                double roll, pitch, yaw;
                tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
                const double yawWeight = 1.0 / std::max<double>(
                    sample.pose.covariance[35], 1e-6);
                yawSinSum += yawWeight * std::sin(yaw);
                yawCosSum += yawWeight * std::cos(yaw);
            }
        }
        const double meanX = xSum / weightSum;
        const double meanY = ySum / weightSum;
        double maxSpread = 0.0;
        for (const auto& sample : rtkInitializationSamples) {
            maxSpread = std::max(
                maxSpread,
                std::hypot(sample.pose.pose.position.x - meanX,
                           sample.pose.pose.position.y - meanY));
        }
        if (maxSpread > rtkInitializationMaxSpread) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "RTK initialization is not stable: spread %.3f m exceeds %.3f m",
                maxSpread, rtkInitializationMaxSpread);
            return false;
        }

        transformTobeMapped[0] = cloudInfo.imu_roll_init;
        transformTobeMapped[1] = cloudInfo.imu_pitch_init;
        transformTobeMapped[2] = rtkUseHeading
            ? static_cast<float>(std::atan2(yawSinSum, yawCosSum))
            : static_cast<float>(init_guess.size() >= 3
                                     ? init_guess[2]
                                     : cloudInfo.imu_yaw_init);
        lastPoses3D.x = transformTobeMapped[3] = static_cast<float>(meanX);
        lastPoses3D.y = transformTobeMapped[4] = static_cast<float>(meanY);
        lastPoses3D.z = transformTobeMapped[5] = init_guess.size() >= 6 ? init_guess[5] : 0.0;
        lastPoses6D = trans2PointTypePose(transformTobeMapped);
        lastPoses6D.time = timeLaserInfoCur;
        LocInitSta = InitializedFlag::Initialized;
        rtkInitializationSamples.clear();
        RCLCPP_INFO(get_logger(),
                    "Localization initialized from %d stable map-aligned RTK samples: x=%.3f y=%.3f yaw=%.3f heading=%s",
                    rtkInitializationMinSamples, transformTobeMapped[3],
                    transformTobeMapped[4], transformTobeMapped[2],
                    rtkUseHeading ? "RTK" : "configured/IMU");
        return true;
    }

    void applyLocalizationRTKGuess() {
        if (!useRTKAssist || LocInitSta != InitializedFlag::Initialized) return;
        nav_msgs::msg::Odometry rtkOdom;
        if (!getLocalizationRTK(rtkOdom)) return;

        float gpsX = rtkOdom.pose.pose.position.x;
        float gpsY = rtkOdom.pose.pose.position.y;
        const float dx = gpsX - transformTobeMapped[3];
        const float dy = gpsY - transformTobeMapped[4];
        const float innovation = std::hypot(dx, dy);
        const float normalizedInnovation = std::sqrt(
            dx * dx / std::max<float>(rtkOdom.pose.covariance[0], gpsVarianceFloor) +
            dy * dy / std::max<float>(rtkOdom.pose.covariance[7], gpsVarianceFloor));
        if ((rtkMaxInnovation > 0.0f && innovation > rtkMaxInnovation) ||
            (rtkMaxNormalizedInnovation > 0.0f &&
             normalizedInnovation > rtkMaxNormalizedInnovation)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Rejecting RTK localization assist: innovation %.3f m (normalized %.2f)",
                                 innovation, normalizedInnovation);
            return;
        }
        float blend = rtkPositionBlend;
        if (rtkAdaptiveBlend) {
            const float worstVariance = std::max<float>(
                rtkOdom.pose.covariance[0], rtkOdom.pose.covariance[7]);
            const float quality = std::clamp(
                1.0f - worstVariance / gpsCovThreshold, 0.0f, 1.0f);
            blend *= quality;
        }
        transformTobeMapped[3] = (1.0f - blend) * transformTobeMapped[3] + blend * gpsX;
        transformTobeMapped[4] = (1.0f - blend) * transformTobeMapped[4] + blend * gpsY;
    }

    void SCReLoc() {
        std::cerr << "[ReLoc] SC Initializing" << std::endl;

        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        copyCornerCloudKeyFrames = cornerCloudKeyFrames;
        copySurfCloudKeyFrames = surfCloudKeyFrames;

        // 使用当前扫描去查询
        downSizeFilterSurf.setInputCloud(laserCloudRaw);
        downSizeFilterSurf.filter(*laserCloudRawDS);

        // find keys
        std::pair<int, float> detectResult;
        {
            std::lock_guard<std::mutex> lock(mtxScanContext);
            detectResult = scManager.detectLoopClosureID(*laserCloudRawDS);
        }
        int loopKeyPre = detectResult.first;
        // float yawDiffRad = detectResult.second;  // not use for v1 (because pcl icp withi initial somthing wrong...)
        if (loopKeyPre == -1) {
            std::cerr << "[ReLoc] SC no found" << std::endl;
            return;
        }

        if (loopKeyPre < 0 || loopKeyPre >= static_cast<int>(idxMap->size())) {
            RCLCPP_ERROR(get_logger(), "Scan Context returned an invalid descriptor index: %d", loopKeyPre);
            return;
        }
        loopKeyPre = static_cast<int>(idxMap->points[loopKeyPre].intensity);
        if (loopKeyPre < 0 || loopKeyPre >= static_cast<int>(cloudKeyPoses6D->size())) {
            RCLCPP_ERROR(get_logger(), "Scan Context index maps to an invalid keyframe: %d", loopKeyPre);
            return;
        }
        std::cout << "[ReLoc] SC loop found: " << detectResult.first << " map to " << loopKeyPre << std::endl;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud = laserCloudRawDS;
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());

        // Build the target submap in the matched keyframe coordinate system.
        if (subMapMode) historyKeyframeSearchNum = 2;
        loopFindNearKeyframesWithRespectTo(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, loopKeyPre);
        if (cureKeyframeCloud->size() < 100 || prevKeyframeCloud->size() < 300) {
            RCLCPP_WARN(get_logger(), "Insufficient points for Scan Context relocalization ICP: source=%zu target=%zu",
                        cureKeyframeCloud->size(), prevKeyframeCloud->size());
            return;
        }
        // 如果不叠加，getFitnessScore分数很高，根本上不去
        // loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 2);                         // giseop
        // loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);  // giseop
        if (pubHistoryKeyFrames->get_subscription_count() != 0) publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);

        // ICP Settings
        auto configureIcp = [&](pcl::IterativeClosestPoint<PointType, PointType>& candidate) {
            candidate.setMaxCorrespondenceDistance(std::max(5.0f, historyKeyframeSearchRadius * 2.0f));
            candidate.setMaximumIterations(100);
            candidate.setTransformationEpsilon(1e-6);
            candidate.setEuclideanFitnessEpsilon(1e-6);
            candidate.setRANSACIterations(0);
            candidate.setInputSource(cureKeyframeCloud);
            candidate.setInputTarget(prevKeyframeCloud);
        };

        // Scan Context's shift convention differs among forks. Test both yaw
        // directions and keep the converged solution with the lower score.
        pcl::IterativeClosestPoint<PointType, PointType> icpPositive;
        pcl::IterativeClosestPoint<PointType, PointType> icpNegative;
        configureIcp(icpPositive);
        configureIcp(icpNegative);
        pcl::PointCloud<PointType> resultPositive;
        pcl::PointCloud<PointType> resultNegative;
        const Eigen::Matrix4f positiveGuess =
            pcl::getTransformation(0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                   detectResult.second).matrix();
        const Eigen::Matrix4f negativeGuess =
            pcl::getTransformation(0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                   -detectResult.second).matrix();
        icpPositive.align(resultPositive, positiveGuess);
        icpNegative.align(resultNegative, negativeGuess);

        pcl::IterativeClosestPoint<PointType, PointType>* bestIcp = nullptr;
        if (icpPositive.hasConverged()) bestIcp = &icpPositive;
        if (icpNegative.hasConverged() &&
            (bestIcp == nullptr || icpNegative.getFitnessScore() < bestIcp->getFitnessScore())) {
            bestIcp = &icpNegative;
        }
        const double bestScore = bestIcp == nullptr
            ? std::numeric_limits<double>::infinity()
            : bestIcp->getFitnessScore();
        if (bestIcp == nullptr || bestScore > historyKeyframeFitnessScore) {
            std::cout << "[ReLoc] ICP fitness test failed (" << bestScore << " > " << historyKeyframeFitnessScore << "). Reject this SC loop."
                      << std::endl;
            return;
        } else {
            std::cout << "[ReLoc] ICP fitness test passed (" << bestScore << " < " << historyKeyframeFitnessScore << "). Add this SC loop."
                      << std::endl;
        }

        // publish corrected cloud
        if (pubIcpKeyFrames->get_subscription_count() != 0) {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, bestIcp->getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = bestIcp->getFinalTransformation();
        Eigen::Affine3f loopKeyPreTransformInTheWorld = pclPointToAffine3f(cloudKeyPoses6D->points[loopKeyPre]);  // zxl
        Eigen::Affine3f loopKeyCurTransformInTheWorld = loopKeyPreTransformInTheWorld * correctionLidarFrame;     // zxl

        // zxl
        pcl::getTranslationAndEulerAngles(loopKeyCurTransformInTheWorld, x, y, z, roll, pitch, yaw);
        transformTobeMapped[0] = roll;
        transformTobeMapped[1] = pitch;
        transformTobeMapped[2] = yaw;
        lastPoses3D.x = transformTobeMapped[3] = x;
        lastPoses3D.y = transformTobeMapped[4] = y;
        lastPoses3D.z = transformTobeMapped[5] = z;

        lastPoses6D = trans2PointTypePose(transformTobeMapped);
        lastPoses6D.time = timeLaserInfoCur;

        std::cout << "[ReLoc is OK] the pose loop is: x" << x << " y" << y << " z" << z << std::endl;

        LocInitSta = InitializedFlag::Initialized;
        publishLocalizationState(true);
    }

    /////////////////////////////////// Loc End ///////////////////////////////////

    void laserCloudInfoHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msgIn) {
        if (imageProjection.cloudHandler(msgIn) == false) return;

        if (!featureExtraction.FeatureExtractionHandler(
                imageProjection.cloudInfo)) return;

        cloudInfo = featureExtraction.cloudInfo;

        // The loop-closure worker snapshots these values while holding mtx.
        // Keep their updates in the same critical section to avoid a data race.
        std::lock_guard<std::mutex> lock(mtx);
        hasValidLidarInput = true;
        hasProcessedLidar = true;

        // extract time stamp
        timeLaserInfoStamp = cloudInfo.header.stamp;
        timeLaserInfoCur = stamp2Sec(cloudInfo.header.stamp);

        // extract info and feature cloud
        // pcl::fromROSMsg(cloudInfo.cloud_corner, *laserCloudCornerLast);
        // pcl::fromROSMsg(cloudInfo.cloud_surface, *laserCloudSurfLast);
        // pcl::fromROSMsg(cloudInfo.cloud_deskewed, *laserCloudRaw);
        laserCloudCornerLast = cloudInfo.cloud_corner;
        laserCloudSurfLast = cloudInfo.cloud_surface;
        laserCloudRaw = cloudInfo.cloud_deskewed;

        static double timeLastProcessing = -1;
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval) {
            timeLastProcessing = timeLaserInfoCur;

            auto t0 = GET_TIME();

            updateInitialGuess();

            if (LocEnableFlag) {
                publishLocalizationState();
                if (LocInitSta != InitializedFlag::Initialized) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "Localization is not initialized yet, keep relocalizing. attempts=%d",
                                         relocalizationAttemptCount);
                    return;
                }
            }
            if (LocEnableFlag) applyLocalizationRTKGuess();

            auto t1 = GET_TIME();

            if (LocEnableFlag)
                extractNearbyLoc();
            else
                extractSurroundingKeyFrames();

            downsampleCurrentScan();

            auto t3 = GET_TIME();

            const bool scanMatchOk = scan2MapOptimization();
            if (LocEnableFlag) {
                if (scanMatchOk) {
                    localizationBadMatchCount = 0;
                    ++localizationConsecutiveSuccesses;
                } else {
                    localizationConsecutiveSuccesses = 0;
                    ++localizationBadMatchCount;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "Localization scan-to-map failed, skip this odometry frame. bad_count=%d/%d",
                                         localizationBadMatchCount, lostBadMatchThreshold);
                    if (lostDetectionEnable && localizationBadMatchCount >= lostBadMatchThreshold) {
                        LocInitSta = InitializedFlag::MayLost;
                        localizationLossEventPending = true;
                        publishLocalizationState(true);
                        LocInitSta = InitializedFlag::NonInitialized;
                        localizationBadMatchCount = 0;
                        localizationConsecutiveSuccesses = 0;
                        dynamicFilterFrameCount = 0;
                        rtkInitializationSamples.clear();
                        resetInitialGuessSeed = true;
                        RCLCPP_ERROR(get_logger(), "Localization lost, switch back to relocalizing");
                    }
                    return;
                }
            }

            // In simulation the Gazebo odometry pose is authoritative. Use
            // it again after scan matching so weak railway geometry cannot
            // pull the registered cloud away from the simulated chassis.
            if (applyExternalPoseOverride()) {
                incrementalOdometryAffineBack =
                    trans2Affine3f(transformTobeMapped);
            }

            auto t4 = GET_TIME();

            if (LocEnableFlag)
                saveKeyFramesAndLoc();
            else {
                saveKeyFramesAndFactor();

                correctPoses();
            }

            publishOdometry();

            // This is an algorithm input for VIS depth registration, not an
            // RViz-only diagnostic topic.
            publishCloud(pubExtractedCloud, cloudInfo.cloud_deskewed,
                         cloudInfo.header.stamp, lidarFrame);

            if (useRviz) {
                publishFrames();

                publishCloud(pubCornerPoints, cloudInfo.cloud_corner, cloudInfo.header.stamp, lidarFrame);
                publishCloud(pubSurfacePoints, cloudInfo.cloud_surface, cloudInfo.header.stamp, lidarFrame);
            }

            auto t5 = GET_TIME();

            printf("ext: %.2f icp: %.2f all: %.2f ", GET_USED(t3, t1) * 1000, GET_USED(t4, t3) * 1000, GET_USED(t5, t0) * 1000);
            printf("| ds: %d + %d map: %d + %d", laserCloudSurfLastDSNum, laserCloudCornerLastDSNum, laserCloudSurfFromMapDSNum, laserCloudCornerFromMapDSNum);
            std::cout << std::endl;
        }
    }

    void gpsHandler(const nav_msgs::msg::Odometry::SharedPtr gpsMsg) {
        std::lock_guard<std::mutex> lock(mtxGPS);
        const double timestamp = stamp2Sec(gpsMsg->header.stamp);
        if (!std::isfinite(timestamp) || timestamp <= lastGpsInputTime) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Discarding RTK/GPS sample with invalid or non-increasing timestamp");
            return;
        }
        lastGpsInputTime = timestamp;
        const std::size_t queueLimit = static_cast<std::size_t>(
            std::max(1, gpsQueueSize));
        if (useGpsFactor) gpsQueue.push_back(*gpsMsg);
        while (gpsQueue.size() > queueLimit) gpsQueue.pop_front();
        if (LocEnableFlag && useRTKAssist)
            localizationRtkQueue.push_back(*gpsMsg);
        while (localizationRtkQueue.size() > queueLimit)
            localizationRtkQueue.pop_front();
    }

    void externalPoseHandler(const nav_msgs::msg::Odometry::SharedPtr poseMsg) {
        std::lock_guard<std::mutex> lock(mtxExternalPose);
        const double timestamp = stamp2Sec(poseMsg->header.stamp);
        const auto& position = poseMsg->pose.pose.position;
        const auto& orientation = poseMsg->pose.pose.orientation;
        const double quaternionNorm = std::sqrt(
            orientation.x * orientation.x + orientation.y * orientation.y +
            orientation.z * orientation.z + orientation.w * orientation.w);
        if (!std::isfinite(timestamp) || !std::isfinite(position.x) ||
            !std::isfinite(position.y) || !std::isfinite(position.z) ||
            !std::isfinite(quaternionNorm) || quaternionNorm < 1e-6 ||
            (latestExternalPoseAvailable &&
             timestamp <= stamp2Sec(latestExternalPose.header.stamp))) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Discarding external pose with invalid data or non-increasing timestamp");
            return;
        }
        externalPoseQueue.push_back(*poseMsg);
        while (externalPoseQueue.size() > 200) externalPoseQueue.pop_front();
        latestExternalPose = *poseMsg;
        latestExternalPoseAvailable = true;
    }

    void pointAssociateToMap(PointType const* const pi, PointType* const po) {
        po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y + transPointAssociateToMap(0, 2) * pi->z +
                transPointAssociateToMap(0, 3);
        po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y + transPointAssociateToMap(1, 2) * pi->z +
                transPointAssociateToMap(1, 3);
        po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y + transPointAssociateToMap(2, 2) * pi->z +
                transPointAssociateToMap(2, 3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn) {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur =
            pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i) {
            const auto& pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, Eigen::Affine3f transCur) {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i) {
            const auto& pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint) {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                            gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[]) {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint) {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[]) {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[]) {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw = transformIn[2];
        return thisPose6D;
    }

    void savePath(const nav_msgs::msg::Path& path, const std::string& file_path) {
        std::ofstream file(file_path, std::ios::out);
        if (!file.is_open()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open file: " << file_path);
            return;
        }
        file.setf(std::ios::fixed, std::ios::floatfield);
        file.precision(6);

        for (const auto& pose_stamped : path.poses) {
            const auto& pos = pose_stamped.pose.position;
            const auto& q = pose_stamped.pose.orientation;
            file << stamp2Sec(pose_stamped.header.stamp) << " " << pos.x << " " << pos.y << " " << pos.z << " " << q.x << " " << q.y << " " << q.z << " "
                 << q.w << "\n";
        }

        file.close();
    }

    void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename) {
        using namespace gtsam;

        // ref from gtsam's original code "dataset.cpp"
        std::fstream stream(_filename.c_str(), fstream::out);

        for (const auto& key_value : _estimates) {
            auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
            if (!p) continue;

            const Pose3& pose = p->value();

            Point3 t = pose.translation();
            Rot3 R = pose.rotation();
            auto col1 = R.column(1);  // Point3
            auto col2 = R.column(2);  // Point3
            auto col3 = R.column(3);  // Point3

            stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " " << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y()
                   << " " << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
        }
    }
    void visualizeGlobalMapThread() {
        rclcpp::Rate rate(0.2);
        while (rclcpp::ok()) {
            rate.sleep();
            if (LocEnableFlag == false) publishGlobalMap();  // 定位模式，不使用
        }
    }

    // Called by main only after the executor and all worker threads have
    // stopped. This avoids serializing mutable graph/keyframe containers while
    // loop closure is still updating them.
    void saveMapOnShutdown() {
        if (savePCD == false) return;
        const std::size_t keyframeCount = cloudKeyPoses3D->size();
        if (keyframeCount == 0 || cloudKeyPoses6D->size() != keyframeCount ||
            cornerCloudKeyFrames.size() != keyframeCount ||
            surfCloudKeyFrames.size() != keyframeCount) {
            RCLCPP_ERROR(
                get_logger(),
                "Refusing final map serialization: inconsistent keyframe containers "
                "(pose3d=%zu pose6d=%zu corner=%zu surface=%zu)",
                keyframeCount, cloudKeyPoses6D->size(),
                cornerCloudKeyFrames.size(), surfCloudKeyFrames.size());
            return;
        }
        if (mapArtifactWriteFailed) {
            RCLCPP_ERROR(
                get_logger(),
                "Refusing final map serialization: one or more per-keyframe "
                "PCD/SCD files failed to write during mapping");
            return;
        }
        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;

        const std::string kitti_format_pg_filename{savePCDDirectory + "optimized_poses.txt"};
        saveOptimizedVerticesKITTIformat(isamCurrentEstimate, kitti_format_pg_filename);
        savePath(globalPath, savePCDDirectory + "traj_tum.txt");
        const bool trajectorySaved = savePCDIfNotEmpty(
            savePCDDirectory + "trajectory.pcd", *cloudKeyPoses3D, false);
        const bool transformationsSaved = savePCDIfNotEmpty(
            savePCDDirectory + "transformations.pcd", *cloudKeyPoses6D,
            false);
        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
            *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
        }
        downSizeFilterCorner.setInputCloud(globalCornerCloud);
        downSizeFilterCorner.filter(*globalCornerCloudDS);
        savePCDIfNotEmpty(savePCDDirectory + "cloudCorner.pcd",
                          *globalCornerCloudDS);
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        savePCDIfNotEmpty(savePCDDirectory + "cloudSurf.pcd",
                          *globalSurfCloudDS);
        *globalMapCloud += *globalCornerCloud;
        *globalMapCloud += *globalSurfCloud;
        const bool globalMapSaved = savePCDIfNotEmpty(
            savePCDDirectory + "cloudGlobal.pcd", *globalMapCloud);
        if (!trajectorySaved || !transformationsSaved || !globalMapSaved) {
            RCLCPP_ERROR(
                get_logger(),
                "Map serialization failed; map_manifest.yaml was not updated");
            return;
        }
        if (!writeMapManifest()) {
            RCLCPP_ERROR(
                get_logger(),
                "Map data was written, but the manifest could not be completed");
            return;
        }
        cout << "****************************************************" << endl;

        cout << "Saving map to pcd files completed" << endl;
    }

    void publishGlobalMap() {
        if (pubLaserCloudSurround->get_subscription_count() == 0) return;

        pcl::PointCloud<PointType>::Ptr poseSnapshot3D(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointTypePose>::Ptr poseSnapshot6D(new pcl::PointCloud<PointTypePose>());
        vector<pcl::PointCloud<PointType>::Ptr> cornerSnapshot;
        vector<pcl::PointCloud<PointType>::Ptr> surfSnapshot;
        rclcpp::Time mapStamp;
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (cloudKeyPoses3D->empty()) return;
            *poseSnapshot3D = *cloudKeyPoses3D;
            *poseSnapshot6D = *cloudKeyPoses6D;
            cornerSnapshot = cornerCloudKeyFrames;
            surfSnapshot = surfCloudKeyFrames;
            mapStamp = timeLaserInfoStamp;
        }

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        kdtreeGlobalMap->setInputCloud(poseSnapshot3D);
        kdtreeGlobalMap->radiusSearch(poseSnapshot3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i) globalMapKeyPoses->push_back(poseSnapshot3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;  // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity,
                                                    globalMapVisualizationPoseDensity);  // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for (auto& pt : globalMapKeyPosesDS->points) {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = poseSnapshot3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i) {
            if (pointDistance(globalMapKeyPosesDS->points[i], poseSnapshot3D->back()) > globalMapVisualizationSearchRadius) continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            if (thisKeyInd < 0 || thisKeyInd >= static_cast<int>(poseSnapshot6D->size()) ||
                thisKeyInd >= static_cast<int>(cornerSnapshot.size()) ||
                thisKeyInd >= static_cast<int>(surfSnapshot.size())) continue;
            *globalMapKeyFrames += *transformPointCloud(cornerSnapshot[thisKeyInd], &poseSnapshot6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfSnapshot[thisKeyInd], &poseSnapshot6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;  // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize,
                                                     globalMapVisualizationLeafSize);  // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, mapStamp, odometryFrame);
    }

    void loopClosureThread() {
        if (loopClosureEnableFlag == false) return;
        if (LocEnableFlag == true) return;  // 定位模式，不使用回环检测

        rclcpp::Rate rate(loopClosureFrequency);
        while (rclcpp::ok()) {
            rate.sleep();
            if (LocEnableFlag == true) return;  // 线程可能不同步

            bool snapshotAvailable = false;
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (!cloudKeyPoses3D->empty()) {
                    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
                    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
                    copyCornerCloudKeyFrames = cornerCloudKeyFrames;
                    copySurfCloudKeyFrames = surfCloudKeyFrames;
                    loopSnapshotStamp = timeLaserInfoStamp;
                    loopSnapshotTime = timeLaserInfoCur;
                    snapshotAvailable = true;
                }
            }

            if (snapshotAvailable) {
                performLoopClosure();
                if (scanContextLoopEnableFlag) performSCLoopClosure();
                visualizeLoopClosure();
            }
        }
    }

    void loopInfoHandler(const std_msgs::msg::Float64MultiArray::SharedPtr loopMsg) {
        if (loopMsg->data.size() != 2 ||
            !std::isfinite(loopMsg->data[0]) ||
            !std::isfinite(loopMsg->data[1])) return;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        loopInfoVec.push_back(*loopMsg);

        while (loopInfoVec.size() > 5) loopInfoVec.pop_front();
    }

    void performLoopClosure() {
        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false) return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000) return;
            if (pubHistoryKeyFrames->get_subscription_count() != 0) publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, loopSnapshotStamp, odometryFrame);
        }

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) return;

        // publish corrected cloud
        if (pubIcpKeyFrames->get_subscription_count() != 0) {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, loopSnapshotStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;  // pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        std::cout << "RS loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl;  // giseop

        // Add pose constraint
        {
            std::lock_guard<std::mutex> lock(mtx);
            loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
            loopPoseQueue.push_back(poseFrom.between(poseTo));
            loopNoiseQueue.push_back(constraintNoise);
        }

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    /////////////////////////////////// SC Start ///////////////////////////////////

    // 找到与给定关键帧在一定范围内的所有关键帧，并对其进行下采样
    void loopFindNearKeyframesWithRespectTo(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int _wrt_key) {
        // 提取附近的关键帧
        nearKeyframes->clear();
        const int cloudSize = static_cast<int>(std::min({
            copy_cloudKeyPoses6D->size(), copyCornerCloudKeyFrames.size(),
            copySurfCloudKeyFrames.size()}));
        if (_wrt_key < 0 || _wrt_key >= cloudSize) return;
        const Eigen::Affine3f wrtTransformInverse =
            pclPointToAffine3f(copy_cloudKeyPoses6D->points[_wrt_key]).inverse();
        for (int i = -searchNum; i <= searchNum; ++i) {  // 在给定关键帧的前后searchNum范围内寻找关键帧
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize)  // 如果keyNear超出范围，则继续下一次循环
                continue;
            // 把找到的关键帧转换到给定的关键帧的坐标系下，并添加到nearKeyframes中
            const Eigen::Affine3f relativeTransform = wrtTransformInverse *
                pclPointToAffine3f(copy_cloudKeyPoses6D->points[keyNear]);
            pcl::PointCloud<PointType> transformedCorner;
            pcl::PointCloud<PointType> transformedSurf;
            pcl::transformPointCloud(*copyCornerCloudKeyFrames[keyNear], transformedCorner,
                                     relativeTransform);
            pcl::transformPointCloud(*copySurfCloudKeyFrames[keyNear], transformedSurf,
                                     relativeTransform);
            *nearKeyframes += transformedCorner;
            *nearKeyframes += transformedSurf;
        }

        if (nearKeyframes->empty()) return;

        // 下采样nearKeyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);  // 把nearKeyframes设置为输入点云
        downSizeFilterICP.filter(*cloud_temp);           // 对输入点云进行下采样，并保存结果到cloud_temp
        *nearKeyframes = *cloud_temp;                    // 用下采样后的点云替换原始的nearKeyframes
    }

    void performSCLoopClosure() {
        // find keys
        std::pair<int, float> detectResult;
        {
            std::lock_guard<std::mutex> lock(mtxScanContext);
            detectResult = scManager.detectLoopClosureID();
        }
        const int cloudSize = static_cast<int>(std::min({
            copy_cloudKeyPoses3D->size(), copy_cloudKeyPoses6D->size(),
            copyCornerCloudKeyFrames.size(), copySurfCloudKeyFrames.size()}));
        if (cloudSize < 2) return;
        int loopKeyCur = cloudSize - 1;
        int loopKeyPre = detectResult.first;
        if (loopKeyPre == -1)  // No loop found
            return;
        if (loopKeyPre < 0 || loopKeyPre >= cloudSize) {
            RCLCPP_WARN(get_logger(),
                        "Ignoring out-of-range Scan Context loop index %d (size=%d)",
                        loopKeyPre, cloudSize);
            return;
        }
        if (loopIndexContainer.find(loopKeyCur) != loopIndexContainer.end()) return;

        std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl;  // giseop

        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            // Use the current pose estimate to place both clouds in the map
            // frame, then let ICP estimate the loop correction.
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000) return;
            if (pubHistoryKeyFrames->get_subscription_count() != 0) publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, loopSnapshotStamp, odometryFrame);
        }

        // ICP Settings
        auto configureIcp = [&](pcl::IterativeClosestPoint<PointType, PointType>& candidate) {
            candidate.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2.0f);
            candidate.setMaximumIterations(100);
            candidate.setTransformationEpsilon(1e-6);
            candidate.setEuclideanFitnessEpsilon(1e-6);
            candidate.setRANSACIterations(0);
            candidate.setInputSource(cureKeyframeCloud);
            candidate.setInputTarget(prevKeyframeCloud);
        };

        const Eigen::Affine3f currentPose =
            pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        const Eigen::Affine3f yawPositive =
            pcl::getTransformation(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, detectResult.second);
        const Eigen::Affine3f yawNegative =
            pcl::getTransformation(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -detectResult.second);
        const Eigen::Matrix4f positiveGuess =
            (currentPose * yawPositive * currentPose.inverse()).matrix();
        const Eigen::Matrix4f negativeGuess =
            (currentPose * yawNegative * currentPose.inverse()).matrix();

        pcl::IterativeClosestPoint<PointType, PointType> icpPositive;
        pcl::IterativeClosestPoint<PointType, PointType> icpNegative;
        configureIcp(icpPositive);
        configureIcp(icpNegative);
        pcl::PointCloud<PointType> resultPositive;
        pcl::PointCloud<PointType> resultNegative;
        icpPositive.align(resultPositive, positiveGuess);
        icpNegative.align(resultNegative, negativeGuess);

        pcl::IterativeClosestPoint<PointType, PointType>* bestIcp = nullptr;
        if (icpPositive.hasConverged()) bestIcp = &icpPositive;
        if (icpNegative.hasConverged() &&
            (bestIcp == nullptr || icpNegative.getFitnessScore() < bestIcp->getFitnessScore())) {
            bestIcp = &icpNegative;
        }
        const double bestScore = bestIcp == nullptr
            ? std::numeric_limits<double>::infinity()
            : bestIcp->getFitnessScore();
        if (bestIcp == nullptr || bestScore > historyKeyframeFitnessScore) {
            std::cout << "ICP fitness test failed (" << bestScore << " > " << historyKeyframeFitnessScore << "). Reject this SC loop." << std::endl;
            return;
        } else {
            std::cout << "ICP fitness test passed (" << bestScore << " < " << historyKeyframeFitnessScore << "). Add this SC loop." << std::endl;
        }

        // publish corrected cloud
        if (pubIcpKeyFrames->get_subscription_count() != 0) {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, bestIcp->getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, loopSnapshotStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = bestIcp->getFinalTransformation();
        const Eigen::Affine3f tWrong =
            pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        const Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

        // giseop, robust kernel for a SC loop
        float robustNoiseScore = std::max(0.05, bestScore);
        gtsam::Vector robustNoiseVector6(6);
        robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
        noiseModel::Base::shared_ptr robustConstraintNoise;
        // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
        robustConstraintNoise =
            gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(1), gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

        // Add pose constraint
        {
            std::lock_guard<std::mutex> lock(mtx);
            loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
            loopPoseQueue.push_back(poseFrom.between(poseTo));
            loopNoiseQueue.push_back(robustConstraintNoise);
        }

        // add loop constriant
        loopIndexContainer.insert(std::pair<int, int>(loopKeyCur, loopKeyPre));  // giseop for multimap
    }

    /////////////////////////////////// SC End ///////////////////////////////////

    bool detectLoopClosureDistance(int* latestID, int* closestID) {
        const int cloudSize = static_cast<int>(std::min(
            copy_cloudKeyPoses3D->size(), copy_cloudKeyPoses6D->size()));
        if (cloudSize < 2) return false;
        int loopKeyCur = cloudSize - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end()) return false;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - loopSnapshotTime) > historyKeyframeSearchTimeDiff) {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int* latestID, int* closestID) {
        double loopTimeCur;
        double loopTimePre;
        {
            std::lock_guard<std::mutex> lock(mtxLoopInfo);
            if (loopInfoVec.empty()) return false;
            loopTimeCur = loopInfoVec.front().data[0];
            loopTimePre = loopInfoVec.front().data[1];
            loopInfoVec.pop_front();
        }

        if (!std::isfinite(loopTimeCur) || !std::isfinite(loopTimePre))
            return false;

        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff) return false;

        const int cloudSize = static_cast<int>(std::min({
            copy_cloudKeyPoses6D->size(), copyCornerCloudKeyFrames.size(),
            copySurfCloudKeyFrames.size()}));
        if (cloudSize < 2) return false;

        auto closest_key = [&](double timestamp) {
            int best_index = -1;
            double best_error = std::numeric_limits<double>::infinity();
            for (int i = 0; i < cloudSize; ++i) {
                const double error =
                    std::abs(copy_cloudKeyPoses6D->points[i].time - timestamp);
                if (error < best_error) {
                    best_error = error;
                    best_index = i;
                }
            }
            return std::make_pair(best_index, best_error);
        };

        const auto current_match = closest_key(loopTimeCur);
        const auto previous_match = closest_key(loopTimePre);
        if (current_match.first < 0 || previous_match.first < 0 ||
            current_match.second > externalLoopTimeTolerance ||
            previous_match.second > externalLoopTimeTolerance)
            return false;

        const int loopKeyCur = current_match.first;
        const int loopKeyPre = previous_match.first;

        if (loopKeyCur == loopKeyPre) return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end()) return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum) {
        // extract near keyframes
        nearKeyframes->clear();
        const int cloudSize = static_cast<int>(std::min({
            copy_cloudKeyPoses6D->size(), copyCornerCloudKeyFrames.size(),
            copySurfCloudKeyFrames.size()}));
        if (key < 0 || key >= cloudSize) return;
        for (int i = -searchNum; i <= searchNum; ++i) {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize) continue;
            *nearKeyframes += *transformPointCloud(copyCornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(copySurfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty()) return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure() {
        if (loopIndexContainer.empty()) return;

        visualization_msgs::msg::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::msg::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = loopSnapshotStamp;
        markerNode.action = visualization_msgs::msg::Marker::ADD;
        markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3;
        markerNode.scale.y = 0.3;
        markerNode.scale.z = 0.3;
        markerNode.color.r = 0;
        markerNode.color.g = 0.8;
        markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::msg::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = loopSnapshotStamp;
        markerEdge.action = visualization_msgs::msg::Marker::ADD;
        markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9;
        markerEdge.color.g = 0.9;
        markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it) {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::msg::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge->publish(markerArray);
    }

    bool applyExternalPoseOverride() {
        if (!useExternalPoseFactor || !externalPoseOverride) return false;

        nav_msgs::msg::Odometry external;
        {
            std::lock_guard<std::mutex> lock(mtxExternalPose);
            if (!latestExternalPoseAvailable) return false;
            external = latestExternalPose;
        }
        const double externalTime = stamp2Sec(external.header.stamp);
        if (std::abs(externalTime - timeLaserInfoCur) > 0.25) return false;

        const auto& position = external.pose.pose.position;
        const auto& orientation = external.pose.pose.orientation;
        if (!std::isfinite(position.x) ||
            !std::isfinite(position.y) ||
            !std::isfinite(position.z) ||
            !std::isfinite(orientation.x) ||
            !std::isfinite(orientation.y) ||
            !std::isfinite(orientation.z) ||
            !std::isfinite(orientation.w)) {
            return false;
        }

        tf2::Quaternion quaternion;
        tf2::fromMsg(orientation, quaternion);
        if (quaternion.length2() < 1e-12) return false;
        quaternion.normalize();
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        transformTobeMapped[0] = static_cast<float>(roll);
        transformTobeMapped[1] = static_cast<float>(pitch);
        transformTobeMapped[2] = static_cast<float>(yaw);
        transformTobeMapped[3] = static_cast<float>(position.x);
        transformTobeMapped[4] = static_cast<float>(position.y);
        transformTobeMapped[5] = static_cast<float>(position.z);
        return true;
    }

    void updateInitialGuess() {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;

        if (resetInitialGuessSeed) {
            lastImuPreTransAvailable = false;
            resetInitialGuessSeed = false;
        }

        // Simulation-only mode: seed every scan with the synchronized
        // simulator pose instead of accumulating an IMU-only motion guess.
        if (applyExternalPoseOverride()) {
            lastImuTransformation = pcl::getTransformation(
                0, 0, 0, cloudInfo.imu_roll_init,
                cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);
            return;
        }

        // SC Reloc
        if (LocEnableFlag && LocInitSta != InitializedFlag::Initialized) {
            LocInitSta = InitializedFlag::Initializing;
            ++relocalizationAttemptCount;
            if (initializeLocalizationFromRTK()) {
                // Map-aligned RTK supplies position and dual-antenna yaw.
            } else if (useSCReLoc) {
                SCReLoc();
            } else {
                // 使用初值
                transformTobeMapped[0] = init_guess[0];
                transformTobeMapped[1] = init_guess[1];
                transformTobeMapped[2] = init_guess[2];
                lastPoses3D.x = transformTobeMapped[3] = init_guess[3];
                lastPoses3D.y = transformTobeMapped[4] = init_guess[4];
                lastPoses3D.z = transformTobeMapped[5] = init_guess[5];

                lastPoses6D = trans2PointTypePose(transformTobeMapped);
                lastPoses6D.time = timeLaserInfoCur;

                LocInitSta = InitializedFlag::Initialized;
            }

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);
            return;
        }

        // initialization
        if (cloudKeyPoses3D->points.empty()) {
            transformTobeMapped[0] = cloudInfo.imu_roll_init;
            transformTobeMapped[1] = cloudInfo.imu_pitch_init;
            transformTobeMapped[2] = cloudInfo.imu_yaw_init;

            if (!useImuHeadingInitialization) transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);
            return;
        }

        // use imu pre-integration estimation for pose guess
        if (cloudInfo.odom_available == true) {
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initial_guess_x, cloudInfo.initial_guess_y, cloudInfo.initial_guess_z,
                                                               cloudInfo.initial_guess_roll, cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);
            if (lastImuPreTransAvailable == false) {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], transformTobeMapped[0],
                                                  transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;

                lastImuTransformation =
                    pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);  // save imu before return;
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imu_available == true) {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], transformTobeMapped[0],
                                              transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation =
                pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);  // save imu before return;
            return;
        }
    }

    // void extractForLoopClosure() {
    //     pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
    //     int numPoses = cloudKeyPoses3D->size();
    //     for (int i = numPoses - 1; i >= 0; --i) {
    //         if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
    //             cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
    //         else
    //             break;
    //     }

    //     extractCloud(cloudToExtract);
    // }

    void extractNearbyLoc() {
        // extract all the nearby key poses and downsample them
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurroundingKeyPoses->radiusSearch(lastPoses3D, (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        const auto maxNearbyKeyframes = static_cast<std::size_t>(surroundingKeyframeSearchMaxNum);
        if (pointSearchInd.size() > maxNearbyKeyframes) pointSearchInd.resize(maxNearbyKeyframes);  // 大于Num进行截断

        // 去掉了降采样，挺没必要的操作

        // 缓存，当前后两次地图一样时，使用上次的地图
        static std::vector<int> last_searchInd;
        std::sort(pointSearchInd.begin(), pointSearchInd.end());  // 首先对两个向量进行排序

        if (last_searchInd.size()) {
            std::vector<int> extra_in_cur;
            std::vector<int> missing_in_cur;
            // 计算 searchInd 相对于 last_searchInd 多出的元素
            std::set_difference(pointSearchInd.begin(), pointSearchInd.end(), last_searchInd.begin(), last_searchInd.end(), std::back_inserter(extra_in_cur));
            // 计算 searchInd 相对于 last_searchInd 少了的元素
            std::set_difference(last_searchInd.begin(), last_searchInd.end(), pointSearchInd.begin(), pointSearchInd.end(), std::back_inserter(missing_in_cur));
            // 如果仅缺少一个，也没必要重构，补上去
            if (extra_in_cur.empty() && missing_in_cur.size() == 1) pointSearchInd.push_back(missing_in_cur[0]);
            // 查看前后两次缓存是否一样
            if (std::equal(pointSearchInd.begin(), pointSearchInd.end(), last_searchInd.begin())) {
                // 如果相同，使用上次的缓存
            } else {
                extractCloud(pointSearchInd);  // 重新累计地图点云
            }
        } else {
            extractCloud(pointSearchInd);  // 初始
        }
        // 更新
        last_searchInd = pointSearchInd;
    }

    void extractNearby() {
        // extract all the nearby key poses and downsample them
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);  // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);

        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)pointSearchInd.size(); ++i) {
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchInd[i]]);
        }

        std::vector<int> extractInd;
        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPoses);
        for (auto& pt : surroundingKeyPoses->points) {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            extractInd.push_back(pointSearchInd[0]);
        }

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses - 1; i >= 0; --i) {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                extractInd.push_back(cloudKeyPoses3D->points[i].intensity);
            else
                break;
        }

        extractCloud(extractInd);
    }

    void extractCloud(std::vector<int> cloudToExtract) {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (auto& thisKeyInd : cloudToExtract) {
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) {
                // transformed cloud available
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }

        auto t1 = GET_TIME();

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000) laserCloudMapContainer.clear();

        auto t2 = GET_TIME();
        printf("ds: %.2f ", GET_USED(t2, t1) * 1000);
    }

    void extractSurroundingKeyFrames() {
        if (cloudKeyPoses3D->points.empty() == true) return;

        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();
        // } else {
        //     extractNearby();
        // }

        extractNearby();
    }

    void downsampleCurrentScan() {
        // giseop
        laserCloudRawDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudRaw);
        downSizeFilterSurf.filter(*laserCloudRawDS);

        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap() { transPointAssociateToMap = trans2Affine3f(transformTobeMapped); }

    bool prepareLocalizationDynamicFilter() {
        if (!LocEnableFlag || LocInitSta != InitializedFlag::Initialized || !dynamicFilterEnable) return false;

        laserCloudCornerLastDSFiltered->clear();
        laserCloudSurfLastDSFiltered->clear();
        dynamicFilterKeptPoints->clear();
        dynamicFilterRejectedPoints->clear();

        const bool inWarmup = dynamicFilterFrameCount < std::max(0, dynamicFilterWarmupFrames);
        const float maxDistance = std::max(0.01f, inWarmup ? dynamicFilterInitialMaxMapDistance : dynamicFilterMaxMapDistance);
        const float maxDistanceSquared = maxDistance * maxDistance;

        updatePointAssociateToMap();

        auto filterCloud = [&](const pcl::PointCloud<PointType>::Ptr& input,
                               const pcl::KdTreeFLANN<PointType>::Ptr& mapTree,
                               const pcl::PointCloud<PointType>::Ptr& filtered) {
            std::vector<int> nearestIndex(1);
            std::vector<float> nearestSquaredDistance(1);

            for (const auto& point : input->points) {
                PointType pointInMap{};
                pointAssociateToMap(&point, &pointInMap);

                const bool matchesPriorMap =
                    mapTree->nearestKSearch(pointInMap, 1, nearestIndex, nearestSquaredDistance) > 0 &&
                    nearestSquaredDistance[0] <= maxDistanceSquared;

                if (matchesPriorMap) {
                    // Keep original lidar-frame coordinates for scan-to-map optimization.
                    filtered->push_back(point);
                    dynamicFilterKeptPoints->push_back(pointInMap);
                } else {
                    dynamicFilterRejectedPoints->push_back(pointInMap);
                }
            }
        };

        filterCloud(laserCloudCornerLastDS, kdtreeCornerFromMap, laserCloudCornerLastDSFiltered);
        filterCloud(laserCloudSurfLastDS, kdtreeSurfFromMap, laserCloudSurfLastDSFiltered);

        const std::size_t totalInput = laserCloudCornerLastDS->size() + laserCloudSurfLastDS->size();
        const std::size_t totalKept = laserCloudCornerLastDSFiltered->size() + laserCloudSurfLastDSFiltered->size();
        const float keepRatio = totalInput == 0 ? 0.0f : static_cast<float>(totalKept) / static_cast<float>(totalInput);

        if (dynamicFilterPublishDebug) {
            if (pubDynamicFilterKeptPoints->get_subscription_count() != 0)
                publishCloud(pubDynamicFilterKeptPoints, dynamicFilterKeptPoints, timeLaserInfoStamp, odometryFrame);
            if (pubDynamicFilterRejectedPoints->get_subscription_count() != 0)
                publishCloud(pubDynamicFilterRejectedPoints, dynamicFilterRejectedPoints, timeLaserInfoStamp, odometryFrame);

            std_msgs::msg::Float32 ratioMessage;
            ratioMessage.data = keepRatio;
            pubDynamicFilterKeepRatio->publish(ratioMessage);
        }

        // The optimizer itself uses strict greater-than checks, so preserve at least
        // one point beyond its existing minimum when deciding whether filtering is safe.
        const int requiredCornerPoints = std::max(dynamicFilterMinCornerPoints, edgeFeatureMinValidNum + 1);
        const int requiredSurfPoints = std::max(dynamicFilterMinSurfPoints, surfFeatureMinValidNum + 1);
        const float boundedMinKeepRatio = std::max(0.0f, std::min(1.0f, dynamicFilterMinKeepRatio));
        const bool filterIsSafe = keepRatio >= boundedMinKeepRatio &&
                                  static_cast<int>(laserCloudCornerLastDSFiltered->size()) >= requiredCornerPoints &&
                                  static_cast<int>(laserCloudSurfLastDSFiltered->size()) >= requiredSurfPoints;

        if (!filterIsSafe) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Localization dynamic filter fallback: kept %.1f%% (%zu/%zu), corner %zu/%d, surface %zu/%d.",
                                 keepRatio * 100.0f, totalKept, totalInput, laserCloudCornerLastDSFiltered->size(), requiredCornerPoints,
                                 laserCloudSurfLastDSFiltered->size(), requiredSurfPoints);
            return false;
        }

        ++dynamicFilterFrameCount;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Localization dynamic filter active: threshold %.2f m, kept %.1f%% (%zu/%zu).",
                             maxDistance, keepRatio * 100.0f, totalKept, totalInput);
        return true;
    }

    void cornerOptimization() {
        updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5;
                cy /= 5;
                cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax;
                    a12 += ax * ay;
                    a13 += ax * az;
                    a22 += ay * ay;
                    a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5;
                a12 /= 5;
                a13 /= 5;
                a22 /= 5;
                a23 /= 5;
                a33 /= 5;

                matA1.at<float>(0, 0) = a11;
                matA1.at<float>(0, 1) = a12;
                matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12;
                matA1.at<float>(1, 1) = a22;
                matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13;
                matA1.at<float>(2, 1) = a23;
                matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                                      ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                                      ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                    float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

                    float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
                               a012 / l12;

                    float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                               a012 / l12;

                    float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                               a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

#define NUM_MATCH_POINTS 5

    void surfOptimization() {
        updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, NUM_MATCH_POINTS, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, NUM_MATCH_POINTS, 3> matA0;
            Eigen::Matrix<float, NUM_MATCH_POINTS, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[NUM_MATCH_POINTS - 1] < 1.0) {
                for (int j = 0; j < NUM_MATCH_POINTS; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < NUM_MATCH_POINTS; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x + pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                               pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs() {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i) {
            if (laserCloudOriCornerFlag[i] == true) {
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i) {
            if (laserCloudOriSurfFlag[i] == true) {
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount) {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));
        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x +
                        (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y +
                        (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

            float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x +
                        ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

            float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x +
                        (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y +
                        ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;
            // lidar -> camera
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR =
            sqrt(pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) + pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) + pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(pow(matX.at<float>(3, 0) * 100, 2) + pow(matX.at<float>(4, 0) * 100, 2) + pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true;
        }
        return false;
    }

    bool scan2MapOptimization() {
        if (cloudKeyPoses3D->points.empty()) return false;

        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

        const auto originalCornerCloud = laserCloudCornerLastDS;
        const auto originalSurfCloud = laserCloudSurfLastDS;
        const int originalCornerCount = laserCloudCornerLastDSNum;
        const int originalSurfCount = laserCloudSurfLastDSNum;
        const bool useDynamicFilteredFeatures = prepareLocalizationDynamicFilter();

        if (useDynamicFilteredFeatures) {
            laserCloudCornerLastDS = laserCloudCornerLastDSFiltered;
            laserCloudSurfLastDS = laserCloudSurfLastDSFiltered;
            laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();
            laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
        }

        bool optimizationRan = false;
        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum) {
            optimizationRan = true;
            for (int iterCount = 0; iterCount < 20; iterCount++) {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true) break;
            }

        } else {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Not enough features! Only %d edge and %d planar features available.",
                laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }

        // Filtering is only an internal localization input. Restore the full feature
        // clouds so mapping/debug publishers and GSeg3D-facing full cloud stay unchanged.
        laserCloudCornerLastDS = originalCornerCloud;
        laserCloudSurfLastDS = originalSurfCloud;
        laserCloudCornerLastDSNum = originalCornerCount;
        laserCloudSurfLastDSNum = originalSurfCount;

        if (!optimizationRan) return false;

        transformUpdate();
        return true;
    }

    void transformUpdate() {
        if (cloudInfo.imu_available == true) {
            if (std::abs(cloudInfo.imu_pitch_init) < 1.4) {
                double imuWeight = imuRPYWeight;
                tf2::Quaternion imuQuaternion;
                tf2::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imu_roll_init, 0, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imu_pitch_init, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit) {
        if (value < -limit) value = -limit;
        if (value > limit) value = limit;

        return value;
    }

    bool saveFrame() {
        if (cloudKeyPoses3D->points.empty()) return true;

        // TODO:
        if (sensor == SensorType::LIVOX) {
            if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0) return true;
        }

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], transformTobeMapped[0],
                                                            transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll) < surroundingkeyframeAddingAngleThreshold && abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
            abs(yaw) < surroundingkeyframeAddingAngleThreshold && sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor() {
        if (cloudKeyPoses3D->points.empty()) {
            noiseModel::Diagonal::shared_ptr priorNoise =
                noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished());  // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        } else {
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    void addGPSFactor() {
        if (!useGpsFactor) return;
        std::lock_guard<std::mutex> gpsLock(mtxGPS);
        if (gpsQueue.empty()) return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else {
            if (gpsInitialDistance > 0.0f &&
                pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) <
                    gpsInitialDistance)
                return;
        }

        // pose covariance small, no need to correct
        if (!gpsFactorAlwaysUse &&
            poseCovariance(3, 3) < poseCovThreshold &&
            poseCovariance(4, 4) < poseCovThreshold) return;

        // last gps position
        static PointType lastGPSPoint;
        static bool hasLastGPSPoint = false;

        while (!gpsQueue.empty()) {
            if (stamp2Sec(gpsQueue.front().header.stamp) <
                timeLaserInfoCur - gpsTimeTolerance) {
                // message too old
                gpsQueue.pop_front();
            } else if (stamp2Sec(gpsQueue.front().header.stamp) >
                       timeLaserInfoCur + gpsTimeTolerance) {
                // message too new
                break;
            } else {
                nav_msgs::msg::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                std::string rejectionReason;
                if (!validateMapAlignedRTK(
                        thisGPS, false, &rejectionReason)) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Rejecting GPS factor input: %s",
                        rejectionReason.c_str());
                    continue;
                }

                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation) {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                if (!std::isfinite(gps_z) || !std::isfinite(noise_z) ||
                    noise_z <= 0.0f) {
                    if (useGpsElevation) continue;
                    gps_z = transformTobeMapped[5];
                    noise_z = gpsVarianceFloor;
                }

                const float positionInnovation = std::hypot(
                    gps_x - transformTobeMapped[3],
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
        // Persist Scan Context descriptors for every saved mapping session,
        // even when online loop closure is disabled. Relocalization consumes
        // these files later and must not depend on the mapping-time detector
        // switch.
        if (loopClosureEnableFlag || savePCD) {
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
                const std::string scdPath =
                    savePCDDirectory + "SCDs/" + curr_scd_node_idx + ".scd";
                if (!saveSCD(scdPath, curr_scd)) {
                    mapArtifactWriteFailed = true;
                    RCLCPP_ERROR(get_logger(), "Failed to save SCD: %s", scdPath.c_str());
                }
            }
        }

        if (savePCD) {
            const bool cornerSaved = savePCDIfNotEmpty(
                savePCDDirectory + "CornerMap/" + std::to_string(curcnt) + ".pcd",
                *thisCornerKeyFrame);
            const bool surfaceSaved = savePCDIfNotEmpty(
                savePCDDirectory + "SurfMap/" + std::to_string(curcnt) + ".pcd",
                *thisSurfKeyFrame);
            const bool scanSaved = savePCDIfNotEmpty(
                savePCDDirectory + "Scans/" + std::to_string(curcnt) + ".pcd",
                *laserCloudRawDS);
            if (!cornerSaved || !surfaceSaved || !scanSaved) {
                mapArtifactWriteFailed = true;
            }
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
            ++imuPreintegrationResetId;
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
        laserOdometryROS.child_frame_id = lidarFrame;
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        laserOdometryROS.pose.pose.orientation = quat_msg;
        laserOdometryROS.pose.covariance[
            lvi_sam::internal_odom_metadata::mapping_correction::kResetId] =
            static_cast<double>(imuPreintegrationResetId);
        pubLaserOdometryGlobal->publish(laserOdometryROS);

        // Publish TF
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        tf2::Transform t_odom_to_lidar = tf2::Transform(quat_tf, tf2::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf2::TimePoint time_point = tf2_ros::fromRclcpp(timeLaserInfoStamp);
        tf2::Stamped<tf2::Transform> temp_odom_to_lidar(t_odom_to_lidar, time_point, odometryFrame);
        geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
        tf2::convert(temp_odom_to_lidar, trans_odom_to_lidar);
        trans_odom_to_lidar.child_frame_id = lidarFrame;
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
            laserOdomIncremental.child_frame_id = lidarFrame;
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);
            laserOdomIncremental.pose.pose.orientation = quat_msg;
        }
        // Internal correction metadata consumed by IMU preintegration.
        // Keep it current even on the first publication and after loop/GPS
        // graph corrections. The two fields intentionally have different
        // semantics from the public Odometry covariance matrix.
        laserOdomIncremental.pose.covariance[
            lvi_sam::internal_odom_metadata::mapping_correction::kResetId] =
            static_cast<double>(imuPreintegrationResetId);
        laserOdomIncremental.pose.covariance[
            lvi_sam::internal_odom_metadata::mapping_correction::kDegenerate] =
            isDegenerate ? 1.0 : 0.0;
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

    auto MO = std::make_shared<MapOptimization>(options);
    exec.add_node(MO);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Map Optimization Started.\033[0m");

    std::thread loopthread(&MapOptimization::loopClosureThread, MO);
    std::thread visualizeMapThread(&MapOptimization::visualizeGlobalMapThread, MO);

    exec.spin();

    rclcpp::shutdown();

    loopthread.join();
    visualizeMapThread.join();
    MO->saveMapOnShutdown();

    return 0;
}
