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
            savePCDDirectory = std::string(home) ã^¶ÖÚ$z{-®éÜj×–æFW„g&öÒÒÆö÷–æFW…VWVU¶•Òæf—'7C°Ğ¢–çB–æFW…FòÒÆö÷–æFW…VWVU¶•Òç6V6öæC°Ğ¢wG6Ó£¥÷6S2÷6T&WGvVVâÒÆö÷÷6UVWVU¶•Ó°Ğ¢WFòæö—6T&WGvVVâÒÆö÷æö—6UVWVU¶•Ó°Ğ¢wE4Öw&‚æFB„&WGvVVäf7F÷#Å÷6S3â†–æFW„g&öÒÂ–æFW…FòÂ÷6T&WGvVVâÂæö—6T&WGvVVâ’“°Ğ¢ĞĞ Ğ¢Æö÷–æFW…VWVRæ6ÆV"‚“°Ğ¢Æö÷÷6UVWVRæ6ÆV"‚“°Ğ¢Æö÷æö—6UVWVRæ6ÆV"‚“°Ğ¢Æö÷—46Æ÷6VBÒG'VS°Ğ¢ĞĞ Ğ¢fö–B6fT¶W”g&ÖW4æDf7F÷"‚’°Ğ¢–b‡6fTg&ÖR‚’ÓÒfÇ6R’&WGW&ã°Ğ Ğ¢òòöFöÒf7F÷ Ğ¢FDöFöÔf7F÷"‚“°Ğ Ğ¢òòw2f7F÷ Ğ¢FDu4f7F÷"‚“°Ğ Ğ¢òò÷F–öæÂ6–×VÆF÷"òv†VVÂÖöFöÖWG'’÷6Rf7F÷"âF†—2—2¶W@Ğ¢òò6W&FRg&öÒF†R%D²òu2F‚æB—2F—6&ÆVB'’FVfVÇBàĞ¢FDW‡FW&æÅ÷6Tf7F÷"‚“°Ğ Ğ¢òòÆö÷f7F÷ Ğ¢FDÆö÷f7F÷"‚“°Ğ Ğ¢òò6÷WBÃÂ"¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢"ÃÂVæFÃ°Ğ¢òòwE4Öw&‚ç&–çB‚$uE4Òw&ƒ¥Æâ"“°Ğ Ğ¢òòWFFR•4ĞĞ¢—6ÒÓçWFFR†wE4Öw&‚Â–æ—F–ÄW7F–ÖFR“°Ğ¢—6ÒÓçWFFR‚“°Ğ Ğ¢–b†Æö÷—46Æ÷6VBÓÒG'VR’°Ğ¢—6ÒÓçWFFR‚“°Ğ¢—6ÒÓçWFFR‚“°Ğ¢—6ÒÓçWFFR‚“°Ğ¢—6ÒÓçWFFR‚“°Ğ¢—6ÒÓçWFFR‚“°Ğ¢ĞĞ Ğ¢wE4Öw&‚ç&W6—¦Rƒ“°Ğ¢–æ—F–ÄW7F–ÖFRæ6ÆV"‚“°Ğ Ğ¢òò6fR¶W’÷6W0Ğ¢ö–çEG—RF†—5÷6S4C°Ğ¢ö–çEG—U÷6RF†—5÷6SdC°Ğ¢÷6S2ÆFW7DW7F–ÖFS°Ğ Ğ¢—6Ô7W'&VçDW7F–ÖFRÒ—6ÒÓæ6Æ7VÆFTW7F–ÖFR‚“°Ğ¢ÆFW7DW7F–ÖFRÒ—6Ô7W'&VçDW7F–ÖFRæCÅ÷6S3â†—6Ô7W'&VçDW7F–ÖFRç6—¦R‚’Ò“°Ğ¢òò6÷WBÃÂ"¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢"ÃÂVæFÃ°Ğ¢òò—6Ô7W'&VçDW7F–ÖFRç&–çB‚$7W'&VçBW7F–ÖFS¢"“°Ğ Ğ¢F†—5÷6S4Bç‚ÒÆFW7DW7F–ÖFRçG&ç6ÆF–öâ‚’ç‚‚“°Ğ¢F†—5÷6S4Bç’ÒÆFW7DW7F–ÖFRçG&ç6ÆF–öâ‚’ç’‚“°Ğ¢F†—5÷6S4Bç¢ÒÆFW7DW7F–ÖFRçG&ç6ÆF–öâ‚’ç¢‚“°Ğ¢F†—5÷6S4Bæ–çFVç6—G’Ò6Æ÷VD¶W•÷6W34BÓç6—¦R‚“²òòF†—26â&RW6VB2–æFW€Ğ¢6Æ÷VD¶W•÷6W34BÓçW6…ö&6²‡F†—5÷6S4B“°Ğ Ğ¢F†—5÷6SdBç‚ÒF†—5÷6S4Bçƒ°Ğ¢F†—5÷6SdBç’ÒF†—5÷6S4Bç“°Ğ¢F†—5÷6SdBç¢ÒF†—5÷6S4Bç£°Ğ¢F†—5÷6SdBæ–çFVç6—G’ÒF†—5÷6S4Bæ–çFVç6—G“²òòF†—26â&RW6VB2–æFW€Ğ¢F†—5÷6SdBç&öÆÂÒÆFW7DW7F–ÖFRç&÷FF–öâ‚’ç&öÆÂ‚“°Ğ¢F†—5÷6SdBç—F6‚ÒÆFW7DW7F–ÖFRç&÷FF–öâ‚’ç—F6‚‚“°Ğ¢F†—5÷6SdBç–rÒÆFW7DW7F–ÖFRç&÷FF–öâ‚’ç–r‚“°Ğ¢F†—5÷6SdBçF–ÖRÒF–ÖTÆ6W$–æfô7W#°Ğ¢6Æ÷VD¶W•÷6W3dBÓçW6…ö&6²‡F†—5÷6SdB“°Ğ Ğ¢òò6÷WBÃÂ"¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢"ÃÂVæFÃ°Ğ¢òò6÷WBÃÂ%÷6R6÷f&–æ6S¢"ÃÂVæFÃ°Ğ¢òò6÷WBÃÂ—6ÒÓæÖ&v–æÄ6÷f&–æ6R†—6Ô7W'&VçDW7F–ÖFRç6—¦R‚’Ó’ÃÂVæFÂÃÂVæFÃ°Ğ¢÷6T6÷f&–æ6RÒ—6ÒÓæÖ&v–æÄ6÷f&–æ6R†—6Ô7W'&VçDW7F–ÖFRç6—¦R‚’Ò“°Ğ Ğ¢òò6fRWFFVBG&ç6f÷&ĞĞ¢G&ç6f÷&ÕFö&TÖVE³ÒÒÆFW7DW7F–ÖFRç&÷FF–öâ‚’ç&öÆÂ‚“°Ğ¢G&ç6f÷&ÕFö&TÖVE³ÒÒÆFW7DW7F–ÖFRç&÷FF–öâ‚’ç—F6‚‚“°Ğ¢G&ç6f÷&ÕFö&TÖVE³%ÒÒÆFW7DW7F–ÖFRç&÷FF–öâ‚’ç–r‚“°Ğ¢G&ç6f÷&ÕFö&TÖVE³5ÒÒÆFW7DW7F–ÖFRçG&ç6ÆF–öâ‚’ç‚‚“°Ğ¢G&ç6f÷&ÕFö&TÖVE³EÒÒÆFW7DW7F–ÖFRçG&ç6ÆF–öâ‚’ç’‚“°Ğ¢G&ç6f÷&ÕFö&TÖVE³UÒÒÆFW7DW7F–ÖFRçG&ç6ÆF–öâ‚’ç¢‚“°Ğ Ğ¢òò6fRÆÂF†R&V6V—fVBVFvRæB7W&bö–çG0Ğ¢6Ã£¥ö–çD6Æ÷VCÅö–çEG—Sã£¥G"F†—46÷&æW$¶W”g&ÖR†æWr6Ã£¥ö–çD6Æ÷VCÅö–çEG—Sâ‚’“°Ğ¢6Ã£¥ö–çD6Æ÷VCÅö–çEG—Sã£¥G"F†—57W&d¶W”g&ÖR†æWr6Ã£¥ö–çD6Æ÷VCÅö–çEG—Sâ‚’“°Ğ¢6Ã£¦6÷•ö–çD6Æ÷VB‚¦Æ6W$6Æ÷VD6÷&æW$Æ7DE2Â§F†—46÷&æW$¶W”g&ÖR“°Ğ¢6Ã£¦6÷•ö–çD6Æ÷VB‚¦Æ6W$6Æ÷VE7W&dÆ7DE2Â§F†—57W&d¶W”g&ÖR“°Ğ Ğ¢òò6fR¶W’g&ÖR6Æ÷V@Ğ¢6÷&æW$6Æ÷VD¶W”g&ÖW2çW6…ö&6²‡F†—46÷&æW$¶W”g&ÖR“°Ğ¢7W&d6Æ÷VD¶W”g&ÖW2çW6…ö&6²‡F†—57W&d¶W”g&ÖR“°Ğ Ğ¢òò6fRF‚f÷"f—7VÆ—¦F–öàĞ¢WFFUF‚‡F†—5÷6SdB“°Ğ Ğ¢–çB7W&6çBÒ6Æ÷VD¶W•÷6W34BÓç6—¦R‚’Ò°Ğ¢–b†Æö÷6Æ÷7W&TVæ&ÆTfÆr’°Ğ¢7FC£¦Æö6µöwV&CÇ7FC£¦×WFWƒâ66ä6öçFW‡DÆö6²†×G…66ä6öçFW‡B“°Ğ¢òò66â6öçFW‡BÆö÷FWFV7F÷"Òv—6V÷ Ğ¢òòÒ4”ätÄUõ44åôeTÄÃ¢W6–ærF÷vç6×ÆVB÷&–v–æÂö–çB6Æ÷VB‚ögVÆÅö6Æ÷VE÷&ö¦V7FVB²F÷vç6×Æ–ærĞ¢òòÒ4”ätÄUõ44åôdTC¢W6–ær7W&f6RfVGW&R2â–çWBö–çB6Æ÷VBf÷"66â6öçFW‡Bƒ##ãBã¢6†V6¶VB—Bv÷&·2âĞ¢òòÒÕTÅD•õ44åôdTC¢W6–æræV$¶W–g&ÖW2†&V6W6R×VÅ&â66âFöW2æ÷B†fR&W–öæB&Vv–öâÂ6òFò6öÇfRF†—2—77VRâââĞ¢6öç7B44–çWEG—R65ö–çWE÷G—RÒ44–çWEG—S£¥4”ätÄUõ44åôeTÄÃ²òò6†ævRF†—0Ğ Ğ¢–b‡65ö–çWE÷G—RÓÒ44–çWEG—S£¥4”ätÄUõ44åôeTÄÂ’°Ğ¢64ÖævW"æÖ¶TæE6fU66æ6öçFW‡DæD¶W—2‚¦Æ6W$6Æ÷VE&tE2“°Ğ¢ÒVÇ6R–b‡65ö–çWE÷G—RÓÒ44–çWEG—S£¥4”ätÄUõ44åôdTB’°Ğ¢64ÖævW"æÖ¶TæE6fU66æ6öçFW‡DæD¶W—2‚§F†—57W&d¶W”g&ÖR“°Ğ¢ÒVÇ6R–b‡65ö–çWE÷G—RÓÒ44–çWEG—S£¤ÕTÅD•õ44åôdTB’°Ğ¢6Ã£¥ö–çD6Æ÷VCÅö–çEG—Sã£¥G"×VÇF”¶W”g&ÖTfVGW&T6Æ÷VB†æWr6Ã£¥ö–çD6Æ÷VCÅö–çEG—Sâ‚’“°Ğ¢Æö÷f–æDæV$¶W–g&ÖW2†×VÇF”¶W”g&ÖTfVGW&T6Æ÷VBÂ7W&6çBÂ†—7F÷'”¶W–g&ÖU6V&6„çVÒ“°Ğ¢64ÖævW"æÖ¶TæE6fU66æ6öçFW‡DæD¶W—2‚¦×VÇF”¶W”g&ÖTfVGW&T6Æ÷VB“°Ğ¢ĞĞ Ğ¢òò6fR62FFĞ¢–b‡6fU4B’°Ğ¢6öç7BWFòb7W'%÷66BÒ64ÖævW"ævWD6öç7E&Ve&V6VçE44B‚“°Ğ¢7FC£§7G&–ær7W'%÷66EöæöFUö–G‚ÒE¦W&÷2†7W&6çB“°Ğ¢6fU44B‡6fU4DF—&V7F÷'’²%44G2ò"²7W'%÷66EöæöFUö–G‚²"ç66B"Â7W'%÷66B“°Ğ¢ĞĞ¢ĞĞ Ğ¢–b‡6fU4B’°Ğ¢6fU4D–dæ÷DV×G’€Ğ¢6fU4DF—&V7F÷'’²$6÷&æW$Öò"²7FC£§Fõ÷7G&–ær†7W&6çB’²"ç6B"ÀĞ¢§F†—46÷&æW$¶W”g&ÖR“°Ğ¢6fU4D–dæ÷DV×G’€Ğ¢6fU4DF—&V7F÷'’²%7W&dÖò"²7FC£§Fõ÷7G&–ær†7W&6çB’²"ç6B"ÀĞ¢§F†—57W&d¶W”g&ÖR“°Ğ¢6fU4D–dæ÷DV×G’€Ğ¢6fU4DF—&V7F÷'’²%66ç2ò"²7FC£§Fõ÷7G&–ær†7W&6çB’²"ç6B"ÀĞ¢¦Æ6W$6Æ÷VE&tE2“°Ğ¢ĞĞ¢ĞĞ Ğ¢fö–B6÷'&V7E÷6W2‚’°Ğ¢–b†6Æ÷VD¶W•÷6W34BÓçö–çG2æV×G’‚’’&WGW&ã°Ğ Ğ¢–b†Æö÷—46Æ÷6VBÓÒG'VR’°Ğ¢òò6ÆV"Ö66†PĞ¢Æ6W$6Æ÷VDÖ6öçF–æW"æ6ÆV"‚“°Ğ¢òò6ÆV"F€Ğ¢vÆö&ÅF‚ç÷6W2æ6ÆV"‚“°Ğ¢òòWFFR¶W’÷6W0Ğ¢–çBçVÕ÷6W2Ò—6Ô7W'&VçDW7F–ÖFRç6—¦R‚“°Ğ¢f÷"†–çB’Ò²’ÂçVÕ÷6W3²²¶’’°Ğ¢6Æ÷VD¶W•÷6W34BÓçö–çG5¶•Òç‚Ò—6Ô7W'&VçDW7F–ÖFRæCÅ÷6S3â†’’çG&ç6ÆF–öâ‚’ç‚‚“°Ğ¢6Æ÷VD¶W•÷6W34BÓçö–çG5¶•Òç’Ò—6Ô7W'&VçDW7F–ÖFRæCÅ÷6S3â†’’çG&ç6ÆF–öâ‚’ç’‚“°Ğ¢6Æ÷VD¶W•÷6W34BÓçö–çG5¶•Òç¢Ò—6Ô7W'&VçDW7F–ÖFRæCÅ÷6S3â†’’çG&ç6ÆF–öâ‚’ç¢‚“°Ğ Ğ¢6Æ÷VD¶W•÷6W3dBÓçö–çG5¶•Òç‚Ò6Æ÷VD¶W•÷6W34BÓçö–çG5¶•Òçƒ°Ğ¢6Æ÷VD¶W•÷6W3dBÓçö–çG5¶•Òç’Ò6Æ÷VD¶W•÷6W34BÓçö–çG5¶•Òç“°Ğ¢6Æ÷VD¶W•÷6W3dBÓçö–çG5¶•Òç¢Ò6Æ÷VD¶W•÷6W34BÓçö–çG5¶•Òç£°Ğ¢6Æ÷VD¶W•÷6W3dBÓçö–çG5¶•Òç&öÆÂÒ—6Ô7W'&VçDW7F–ÖFRæCÅ÷6S3â†’’ç&÷FF–öâ‚’ç&öÆÂ‚“°Ğ¢6Æ÷VD¶W•÷6W3dBÓçö–çG5¶•Òç—F6‚Ò—6Ô7W'&VçDW7F–ÖFRæCÅ÷6S3â†’’ç&÷FF–öâ‚’ç—F6‚‚“°Ğ¢6Æ÷VD¶W•÷6W3dBÓçö–çG5¶•Òç–rÒ—6Ô7W'&VçDW7F–ÖFRæCÅ÷6S3â†’’ç&÷FF–öâ‚’ç–r‚“°Ğ Ğ¢WFFUF‚†6Æ÷VD¶W•÷6W3dBÓçö–çG5¶•Ò“°Ğ¢ĞĞ Ğ¢Æö÷—46Æ÷6VBÒfÇ6S°Ğ¢ĞĞ¢ĞĞ Ğ¢fö–BWFFUF‚†6öç7Bö–çEG—U÷6Rb÷6Uö–â’°Ğ¢vVöÖWG'•ö×6w3£¦×6s£¥÷6U7F×VB÷6U÷7F×VC°Ğ¢÷6U÷7F×VBæ†VFW"ç7F×Ò&6Æ7£¥F–ÖR‡÷6Uö–âçF–ÖR¢S’“°Ğ¢÷6U÷7F×VBæ†VFW"æg&ÖUö–BÒöFöÖWG'”g&ÖS°Ğ¢÷6U÷7F×VBç÷6Rç÷6—F–öâç‚Ò÷6Uö–âçƒ°Ğ¢÷6U÷7F×VBç÷6Rç÷6—F–öâç’Ò÷6Uö–âç“°Ğ¢÷6U÷7F×VBç÷6Rç÷6—F–öâç¢Ò÷6Uö–âç£°Ğ¢Fc#£¥VFW&æ–öâ°Ğ¢ç6WE%’‡÷6Uö–âç&öÆÂÂ÷6Uö–âç—F6‚Â÷6Uö–âç–r“°Ğ¢÷6U÷7F×VBç÷6Ræ÷&–VçFF–öâç‚Òç‚‚“°Ğ¢÷6U÷7F×VBç÷6Ræ÷&–VçFF–öâç’Òç’‚“°Ğ¢÷6U÷7F×VBç÷6Ræ÷&–VçFF–öâç¢Òç¢‚“°Ğ¢÷6U÷7F×VBç÷6Ræ÷&–VçFF–öâçrÒçr‚“°Ğ Ğ¢vÆö&ÅF‚ç÷6W2çW6…ö&6²‡÷6U÷7F×VB“°Ğ¢ĞĞ Ğ¢fö–BV&Æ—6„öFöÖWG'’‚’°Ğ¢òòV&Æ—6‚öFöÖWG'’f÷"$õ2†vÆö&ÂĞ¢æeö×6w3£¦×6s£¤öFöÖWG'’Æ6W$öFöÖWG'•$õ3°Ğ¢Æ6W$öFöÖWG'•$õ2æ†VFW"ç7F×ÒF–ÖTÆ6W$–æfõ7F×°Ğ¢Æ6W$öFöÖWG'•$õ2æ†VFW"æg&ÖUö–BÒöFöÖWG'”g&ÖS°Ğ¢Æ6W$öFöÖWG'•$õ2æ6†–ÆEög&ÖUö–BÒ&öFöÕöÖ–ær#°Ğ¢Æ6W$öFöÖWG'•$õ2ç÷6Rç÷6Rç÷6—F–öâç‚ÒG&ç6f÷&ÕFö&TÖVE³5Ó°Ğ¢Æ6W$öFöÖWG'•$õ2ç÷6Rç÷6Rç÷6—F–öâç’ÒG&ç6f÷&ÕFö&TÖVE³EÓ°Ğ¢Æ6W$öFöÖWG'•$õ2ç÷6Rç÷6Rç÷6—F–öâç¢ÒG&ç6f÷&ÕFö&TÖVE³UÓ°Ğ¢Fc#£¥VFW&æ–öâVE÷Fc°Ğ¢VE÷Fbç6WE%’‡G&ç6f÷&ÕFö&TÖVE³ÒÂG&ç6f÷&ÕFö&TÖVE³ÒÂG&ç6f÷&ÕFö&TÖVE³%Ò“°Ğ¢vVöÖWG'•ö×6w3£¦×6s£¥VFW&æ–öâVEö×6s°Ğ¢Fc#£¦6öçfW'B‡VE÷FbÂVEö×6r“°Ğ¢Æ6W$öFöÖWG'•$õ2ç÷6Rç÷6Ræ÷&–VçFF–öâÒVEö×6s°Ğ¢V$Æ6W$öFöÖWG'”vÆö&ÂÓçV&Æ—6‚†Æ6W$öFöÖWG'•$õ2“°Ğ Ğ¢òòV&Æ—6‚D`Ğ¢VE÷Fbç6WE%’‡G&ç6f÷&ÕFö&TÖVE³ÒÂG&ç6f÷&ÕFö&TÖVE³ÒÂG&ç6f÷&ÕFö&TÖVE³%Ò“°Ğ¢Fc#£¥G&ç6f÷&ÒEööFöÕ÷FõöÆ–F"ÒFc#£¥G&ç6f÷&Ò‡VE÷FbÂFc#£¥fV7F÷#2‡G&ç6f÷&ÕFö&TÖVE³5ÒÂG&ç6f÷&ÕFö&TÖVE³EÒÂG&ç6f÷&ÕFö&TÖVE³UÒ’“°Ğ¢Fc#£¥F–ÖUö–çBF–ÖU÷ö–çBÒFc%÷&÷3£¦g&öÕ&6Æ7‡F–ÖTÆ6W$–æfõ7F×“°Ğ¢Fc#£¥7F×VCÇFc#£¥G&ç6f÷&ÓâFV×ööFöÕ÷FõöÆ–F"‡EööFöÕ÷FõöÆ–F"ÂF–ÖU÷ö–çBÂöFöÖWG'”g&ÖR“°Ğ¢vVöÖWG'•ö×6w3£¦×6s£¥G&ç6f÷&Õ7F×VBG&ç5ööFöÕ÷FõöÆ–F#°Ğ¢Fc#£¦6öçfW'B‡FV×ööFöÕ÷FõöÆ–F"ÂG&ç5ööFöÕ÷FõöÆ–F"“°Ğ¢G&ç5ööFöÕ÷FõöÆ–F"æ6†–ÆEög&ÖUö–BÒ&Æ–F%öÆ–æ²#°Ğ¢–b‡V&Æ—6„Ö–ætöFöÕDb’'"Óç6VæEG&ç6f÷&Ò‡G&ç5ööFöÕ÷FõöÆ–F"“°Ğ Ğ¢òòV&Æ—6‚öFöÖWG'’f÷"$õ2†–æ7&VÖVçFÂĞ¢7FF–2&ööÂÆ7D–æ7&TöFöÕV$fÆrÒfÇ6S°Ğ¢7FF–2æeö×6w3£¦×6s£¤öFöÖWG'’Æ6W$öFöÔ–æ7&VÖVçFÃ²òò–æ7&VÖVçFÂöFöÖWG'’×6pĞ¢7FF–2V–vVã£¤ff–æS6b–æ7&TöFöÔff–æS²òò–æ7&VÖVçFÂöFöÖWG'’–âff–æPĞ¢–b†Æ7D–æ7&TöFöÕV$fÆrÓÒfÇ6R’°Ğ¢Æ7D–æ7&TöFöÕV$fÆrÒG'VS°Ğ¢Æ6W$öFöÔ–æ7&VÖVçFÂÒÆ6W$öFöÖWG'•$õ3°Ğ¢–æ7&TöFöÔff–æRÒG&ç3$ff–æS6b‡G&ç6f÷&ÕFö&TÖVB“°Ğ¢ÒVÇ6R°Ğ¢V–vVã£¤ff–æS6bff–æT–æ7&RÒ–æ7&VÖVçFÄöFöÖWG'”ff–æTg&öçBæ–çfW'6R‚’¢–æ7&VÖVçFÄöFöÖWG'”ff–æT&6³°Ğ¢–æ7&TöFöÔff–æRÒ–æ7&TöFöÔff–æR¢ff–æT–æ7&S°Ğ¢fÆöB‚Â’Â¢Â&öÆÂÂ—F6‚Â–s°Ğ¢6Ã£¦vWEG&ç6ÆF–öäæDWVÆW$ævÆW2†–æ7&TöFöÔff–æRÂ‚Â’Â¢Â&öÆÂÂ—F6‚Â–r“°Ğ¢–b†6Æ÷VD–æfòæ–×Uöf–Æ&ÆRÓÒG'VR’°Ğ¢–b‡7FC£¦'2†6Æ÷VD–æfòæ–×U÷—F6…ö–æ—B’ÂãB’°Ğ¢F÷V&ÆR–×UvV–v‡BÒã°Ğ¢Fc#£¥VFW&æ–öâ–×UVFW&æ–öã°Ğ¢Fc#£¥VFW&æ–öâG&ç6f÷&ÕVFW&æ–öã°Ğ¢F÷V&ÆR&öÆÄÖ–BÂ—F6„Ö–BÂ–tÖ–C°Ğ Ğ¢òò6ÆW'&öÆÀĞ¢G&ç6f÷&ÕVFW&æ–öâç6WE%’‡&öÆÂÂÂ“°Ğ¢–×UVFW&æ–öâç6WE%’†6Æ÷VD–æfòæ–×U÷&öÆÅö–æ—BÂÂ“°Ğ¢Fc#£¤ÖG&—ƒ7ƒ2‡G&ç6f÷&ÕVFW&æ–öâç6ÆW'†–×UVFW&æ–öâÂ–×UvV–v‡B’’ævWE%’‡&öÆÄÖ–BÂ—F6„Ö–BÂ–tÖ–B“°Ğ¢&öÆÂÒ&öÆÄÖ–C°Ğ Ğ¢òò6ÆW'—F6€Ğ¢G&ç6f÷&ÕVFW&æ–öâç6WE%’ƒÂ—F6‚Â“°Ğ¢–×UVFW&æ–öâç6WE%’ƒÂ6Æ÷VD–æfòæ–×U÷—F6…ö–æ—BÂ“°Ğ¢Fc#£¤ÖG&—ƒ7ƒ2‡G&ç6f÷&ÕVFW&æ–öâç6ÆW'†–×UVFW&æ–öâÂ–×UvV–v‡B’’ævWE%’‡&öÆÄÖ–BÂ—F6„Ö–BÂ–tÖ–B“°Ğ¢—F6‚Ò—F6„Ö–C°Ğ¢ĞĞ¢ĞĞ¢Æ6W$öFöÔ–æ7&VÖVçFÂæ†VFW"ç7F×ÒF–ÖTÆ6W$–æfõ7F×°Ğ¢Æ6W$öFöÔ–æ7&VÖVçFÂæ†VFW"æg&ÖUö–BÒöFöÖWG'”g&ÖS°Ğ¢Æ6W$öFöÔ–æ7&VÖVçFÂæ6†–ÆEög&ÖUö–BÒ&öFöÕöÖ–ær#°Ğ¢Æ6W$öFöÔ–æ7&VÖVçFÂç÷6Rç÷6Rç÷6—F–öâç‚Òƒ°Ğ¢Æ6W$öFöÔ–æ7&VÖVçFÂç÷6Rç÷6Rç÷6—F–öâç’Ò“°Ğ¢Æ6W$öFöÔ–æ7&VÖVçFÂç÷6Rç÷6Rç÷6—F–öâç¢Ò£°Ğ¢Fc#£¥VFW&æ–öâVE÷Fc°Ğ¢VE÷Fbç6WE%’‡&öÆÂÂ—F6‚Â–r“°Ğ¢vVöÖWG'•ö×6w3£¦×6s£¥VFW&æ–öâVEö×6s°Ğ¢Fc#£¦6öçfW'B‡VE÷FbÂVEö×6r“°Ğ¢Æ6W$öFöÔ–æ7&VÖVçFÂç÷6Rç÷6Ræ÷&–VçFF–öâÒVEö×6s°Ğ¢–b†—4FVvVæW&FRĞ¢Æ6W$öFöÔ–æ7&VÖVçFÂç÷6Ræ6÷f&–æ6U³ÒÒ°Ğ¢VÇ6PĞ¢Æ6W$öFöÔ–æ7&VÖVçFÂç÷6Ræ6÷f&–æ6U³ÒÒ°Ğ¢ĞĞ¢V$Æ6W$öFöÖWG'”–æ7&VÖVçFÂÓçV&Æ—6‚†Æ6W$öFöÔ–æ7&VÖVçFÂ“°Ğ¢ĞĞ Ğ¢fö–BV&Æ—6„g&ÖW2‚’°Ğ¢–b†6Æ÷VD¶W•÷6W34BÓçö–çG2æV×G’‚’’&WGW&ã°Ğ¢òòV&Æ—6‚¶W’÷6W0Ğ¢V&Æ—6„6Æ÷VB‡V$¶W•÷6W2Â6Æ÷VD¶W•÷6W34BÂF–ÖTÆ6W$–æfõ7F×ÂöFöÖWG'”g&ÖR“°Ğ¢òòV&Æ—6‚7W'&÷VæF–ær¶W’g&ÖW0Ğ¢V&Æ—6„6Æ÷VB‡V%&V6VçD¶W”g&ÖW2ÂÆ6W$6Æ÷VE7W&dg&öÔÖE2ÂF–ÖTÆ6W$–æfõ7F×ÂöFöÖWG'”g&ÖR“°Ğ¢òòV&Æ—6‚&Vv—7FW&VB¶W’g&ÖPĞ¢–b‡V%&V6VçD¶W”g&ÖRÓævWE÷7V'67&—F–öåö6÷VçB‚’Ò’°Ğ¢6Ã£¥ö–çD6Æ÷VCÅö–çEG—Sã£¥G"6Æ÷VD÷WB†æWr6Ã£¥ö–çD6Æ÷VCÅö–çEG—Sâ‚’“°Ğ¢ö–çEG—U÷6RF†—5÷6SdBÒG&ç3%ö–çEG—U÷6R‡G&ç6f÷&ÕFö&TÖVB“°Ğ¢¦6Æ÷VD÷WB³Ò§G&ç6f÷&Õö–çD6Æ÷VB†Æ6W$6Æ÷VD6÷&æW$Æ7DE2ÂgF†—5÷6SdB“°Ğ¢¦6Æ÷VD÷WB³Ò§G&ç6f÷&Õö–çD6Æ÷VB†Æ6W$6Æ÷VE7W&dÆ7DE2ÂgF†—5÷6SdB“°Ğ¢V&Æ—6„6Æ÷VB‡V%&V6VçD¶W”g&ÖRÂ6Æ÷VD÷WBÂF–ÖTÆ6W$–æfõ7F×ÂöFöÖWG'”g&ÖR“°Ğ¢ĞĞ¢òòV&Æ—6‚&Vv—7FW&VB†–v‚×&W2&r6Æ÷V@Ğ¢òò–b‡V$6Æ÷VE&Vv—7FW&VE&rÓævWE÷7V'67&—F–öåö6÷VçB‚’Ò’°Ğ¢òò6Ã£¥ö–çD6Æ÷VCÅö–çEG—Sã£¥G"6Æ÷VD÷WB†æWr6Ã£¥ö–çD6Æ÷VCÅö–çEG—Sâ‚’“°Ğ¢òò6Ã£¦g&öÕ$õ4×6r†6Æ÷VD–æfòæ6Æ÷VEöFW6¶WvVBÂ¦6Æ÷VD÷WB“°Ğ¢òòö–çEG—U÷6RF†—5÷6SdBÒG&ç3%ö–çEG—U÷6R‡G&ç6f÷&ÕFö&TÖVB“°Ğ¢òò¦6Æ÷VD÷WBÒ§G&ç6f÷&Õö–çD6Æ÷VB†6Æ÷VD÷WBÂgF†—5÷6SdB“°Ğ¢òòV&Æ—6„6Æ÷VB‡V$6Æ÷VE&Vv—7FW&VE&rÂ6Æ÷VD÷WBÂF–ÖTÆ6W$–æfõ7F×ÂöFöÖWG'”g&ÖR“°Ğ¢òòĞĞ¢òòV&Æ—6‚F€Ğ¢–b‡V%F‚ÓævWE÷7V'67&—F–öåö6÷VçB‚’Ò’°Ğ¢vÆö&ÅF‚æ†VFW"ç7F×ÒF–ÖTÆ6W$–æfõ7F×°Ğ¢vÆö&ÅF‚æ†VFW"æg&ÖUö–BÒöFöÖWG'”g&ÖS°Ğ¢V%F‚ÓçV&Æ—6‚†vÆö&ÅF‚“°Ğ¢ĞĞ¢ĞĞ§Ó°Ğ Ğ¦–çBÖ–â†–çB&v2Â6†"¢¢&wb’°Ğ¢&6Æ7£¦–æ—B†&v2Â&wb“°Ğ Ğ¢&6Æ7£¤æöFT÷F–öç2÷F–öç3°Ğ¢÷F–öç2çW6Uö–çG&÷&ö6W75ö6öÖ×2‡G'VR“°Ğ¢&6Æ7£¦W†V7WF÷'3£¥6–ævÆUF‡&VFVDW†V7WF÷"W†V3°Ğ Ğ¢WFòÔòÒ7FC£¦Ö¶U÷6†&VCÆÖ÷F–Ö—¦F–öãâ†÷F–öç2“°Ğ¢W†V2æFEöæöFR„Ôò“°Ğ Ğ¢$4Ä5ô”ädò‡&6Æ7£¦vWEöÆövvW"‚'&6Æ7"’Â%Ã35³³3&ÒÒÒÒÓâÖ÷F–Ö—¦F–öâ7F'FVBåÃ35³Ò"“°Ğ Ğ¢7FC£§F‡&VBÆö÷F‡&VB‚fÖ÷F–Ö—¦F–öã£¦Æö÷6Æ÷7W&UF‡&VBÂÔò“°Ğ¢7FC£§F‡&VBf—7VÆ—¦TÖF‡&VB‚fÖ÷F–Ö—¦F–öã£§f—7VÆ—¦TvÆö&ÄÖF‡&VBÂÔò“°Ğ Ğ¢W†V2ç7–â‚“°Ğ Ğ¢&6Æ7£§6‡WFF÷vâ‚“°Ğ Ğ¢Æö÷F‡&VBæ¦ö–â‚“°Ğ¢f—7VÆ—¦TÖF‡&VBæ¦ö–â‚“°Ğ Ğ¢&WGW&â°Ğ§ĞĞ 