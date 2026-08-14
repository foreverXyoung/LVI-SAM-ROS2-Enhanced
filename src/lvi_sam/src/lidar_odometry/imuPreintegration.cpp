#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
// #include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
// #include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include "utility.hpp"
#include "lvi_sam/internal_odom_metadata.hpp"
#include "lvi_sam_msgs/msg/localization_reset.hpp"

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

class TransformFusion : public ParamServer {
public:
    static constexpr std::size_t kMaxImuOdomQueueSize = 10000;

    std::mutex mtx;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
    rclcpp::Subscription<lvi_sam_msgs::msg::LocalizationReset>::SharedPtr
        subLocalizationReset;

    rclcpp::CallbackGroup::SharedPtr callbackGroupImuOdometry;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLaserOdometry;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

    Eigen::Isometry3d lidarOdomAffine;
    Eigen::Isometry3d imuOdomAffineFront;
    Eigen::Isometry3d imuOdomAffineBack;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    tf2::Transform configuredLidarToBase;

    double lidarOdomTime = -1;
    deque<nav_msgs::msg::Odometry> imuOdomQueue;
    nav_msgs::msg::Path imuPath;
    double lastPathTime = -1.0;
    std::string lastResetSource;
    std::uint64_t lastResetEventId = 0;
    bool hasLastResetEvent = false;

    explicit TransformFusion(const rclcpp::NodeOptions& options)
        : ParamServer("TransformFusionParamServer", options) {
        if (baseToLidarConfigured) {
            const tf2::Quaternion rotation(
                baseToLidarQuaternion.x(), baseToLidarQuaternion.y(),
                baseToLidarQuaternion.z(), baseToLidarQuaternion.w());
            const tf2::Transform baseToLidar(
                rotation,
                tf2::Vector3(
                    baseToLidarTranslation.x(), baseToLidarTranslation.y(),
                    baseToLidarTranslation.z()));
            configuredLidarToBase = baseToLidar.inverse();
            RCLCPP_INFO(
                get_logger(),
                "Fused base pose will use configured T_base_lidar; TF is not "
                "an estimator input");
        } else if (publishFusedBaseTF && lidarFrame != baselinkFrame) {
            throw std::runtime_error(
                "publishFusedBaseTF requires baseToLidarRotation and "
                "baseToLidarTranslation; physical mounting extrinsics are "
                "configuration inputs and are never inferred from TF");
        }

        callbackGroupImuOdometry = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupLaserOdometry = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto imuOdomOpt = rclcpp::SubscriptionOptions();
        imuOdomOpt.callback_group = callbackGroupImuOdometry;
        auto laserOdomOpt = rclcpp::SubscriptionOptions();
        laserOdomOpt.callback_group = callbackGroupLaserOdometry;

        subLaserOdometry = create_subscription<nav_msgs::msg::Odometry>(
            "lio_sam/mapping/odometry", qos, std::bind(&TransformFusion::lidarOdometryHandler, this, std::placeholders::_1), laserOdomOpt);
        subImuOdometry = create_subscription<nav_msgs::msg::Odometry>(odomTopic + "_incremental", qos_imu,
                                                                      std::bind(&TransformFusion::imuOdometryHandler, this, std::placeholders::_1), imuOdomOpt);
        subLocalizationReset = create_subscription<lvi_sam_msgs::msg::LocalizationReset>(
            localizationResetTopic, rclcpp::QoS(10).reliable(),
            std::bind(&TransformFusion::localizationResetHandler, this,
                      std::placeholders::_1), imuOdomOpt);

        pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic, qos_imu);
        pubImuPath = create_publisher<nav_msgs::msg::Path>("lio_sam/imu/path", qos);

        tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    Eigen::Isometry3d odom2affine(nav_msgs::msg::Odometry odom) {
        tf2::Transform t;
        tf2::fromMsg(odom.pose.pose, t);
        return tf2::transformToEigen(tf2::toMsg(t));
    }

    bool validOdometryPose(const nav_msgs::msg::Odometry& odom) const {
        const double timestamp = stamp2Sec(odom.header.stamp);
        const auto& position = odom.pose.pose.position;
        const auto& orientation = odom.pose.pose.orientation;
        const double quaternionNorm = std::sqrt(
            orientation.x * orientation.x + orientation.y * orientation.y +
            orientation.z * orientation.z + orientation.w * orientation.w);
        return std::isfinite(timestamp) && std::isfinite(position.x) &&
               std::isfinite(position.y) && std::isfinite(position.z) &&
               std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
               std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
               quaternionNorm > 1e-9;
    }

    void lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        std::lock_guard<std::mutex> lock(mtx);

        const double timestamp = stamp2Sec(odomMsg->header.stamp);
        if (!validOdometryPose(*odomMsg) ||
            (lidarOdomTime >= 0.0 && timestamp <= lidarOdomTime)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Discarding invalid or non-monotonic LiDAR odometry");
            return;
        }

        lidarOdomAffine = odom2affine(*odomMsg);
        lidarOdomTime = timestamp;
    }

    void localizationResetHandler(
        const lvi_sam_msgs::msg::LocalizationReset::SharedPtr msg) {
        if (!msg) return;
        std::lock_guard<std::mutex> lock(mtx);
        if (hasLastResetEvent && lastResetSource == msg->source &&
            lastResetEventId == msg->event_id) {
            return;
        }
        lastResetSource = msg->source;
        lastResetEventId = msg->event_id;
        hasLastResetEvent = true;
        if (!msg->reset_imu) return;

        imuOdomQueue.clear();
        lidarOdomTime = -1.0;
        lidarOdomAffine.setIdentity();
        imuPath.poses.clear();
        lastPathTime = -1.0;
        RCLCPP_INFO(
            get_logger(),
            "Applied localization reset event: source=%s reason=%u reset_id=%llu detail=%s",
            msg->source.c_str(), static_cast<unsigned int>(msg->reason),
            static_cast<unsigned long long>(msg->reset_id), msg->detail.c_str());
    }

    void imuOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        std::lock_guard<std::mutex> lock(mtx);

        const double imuOdomTime = stamp2Sec(odomMsg->header.stamp);
        if (!validOdometryPose(*odomMsg) ||
            (!imuOdomQueue.empty() &&
             imuOdomTime <= stamp2Sec(imuOdomQueue.back().header.stamp))) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Discarding invalid or non-monotonic IMU odometry");
            return;
        }

        imuOdomQueue.push_back(*odomMsg);
        if (imuOdomQueue.size() > kMaxImuOdomQueueSize) {
            imuOdomQueue.pop_front();
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "IMU odometry fusion queue reached its limit; dropping oldest sample");
        }

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1) {
            if (requireFreshLidarOdomForTF && !publishImuPredictedOdomWhenUnlocalized) {
                imuOdomQueue.clear();
            }
            return;
        }
        const bool lidarOdomFresh =
            std::abs(imuOdomTime - lidarOdomTime) <= maxLidarOdomAge;
        if (requireFreshLidarOdomForTF && !lidarOdomFresh && !publishImuPredictedOdomWhenUnlocalized) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Skip fused odometry/TF: lidar odometry is stale by %.3f s",
                                 imuOdomTime - lidarOdomTime);
            imuOdomQueue.clear();
            return;
        }
        while (!imuOdomQueue.empty()) {
            if (stamp2Sec(imuOdomQueue.front().header.stamp) <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
        if (imuOdomQueue.empty()) return;
        Eigen::Isometry3d imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Isometry3d imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Isometry3d imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Isometry3d imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        auto t = tf2::eigenToTransform(imuOdomAffineLast);
        tf2::Stamped<tf2::Transform> tCur;
        tf2::convert(t, tCur);

        // publish latest odometry
        nav_msgs::msg::Odometry laserOdometry = imuOdomQueue.back();
        // imuOdomQueue stores an IMU-integrated pose already converted to the
        // physical LiDAR frame. Keep message metadata consistent with the
        // numeric pose; base_link is published separately as an optional TF.
        laserOdometry.child_frame_id = lidarFrame;
        laserOdometry.pose.pose.position.x = t.transform.translation.x;
        laserOdometry.pose.pose.position.y = t.transform.translation.y;
        laserOdometry.pose.pose.position.z = t.transform.translation.z;
        laserOdometry.pose.pose.orientation = t.transform.rotation;
        pubImuOdometry->publish(laserOdometry);

        // TF is an integration output, not an estimator input. In
        // algorithm-only tests it can be disabled without a robot description.
        // The physical LiDAR->base transform is always composed from the
        // selected mounting profile and is never queried from the TF tree.
        if (publishFusedBaseTF && (!requireFreshLidarOdomForTF || lidarOdomFresh)) {
            if (lidarFrame != baselinkFrame) {
                tf2::Stamped<tf2::Transform> tb(
                    tCur * configuredLidarToBase,
                    tf2_ros::fromMsg(odomMsg->header.stamp),
                    odometryFrame);
                tCur = tb;
            }
            geometry_msgs::msg::TransformStamped ts;
            tf2::convert(tCur, ts);
            ts.child_frame_id = baselinkFrame;
            tfBroadcaster->sendTransform(ts);
        } else if (publishFusedBaseTF && requireFreshLidarOdomForTF) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Skip fused TF: lidar odometry is stale by %.3f s",
                                 imuOdomTime - lidarOdomTime);
        }

        // publish IMU path
        double imuTime = stamp2Sec(imuOdomQueue.back().header.stamp);
        if (imuTime - lastPathTime > 0.1) {
            lastPathTime = imuTime;
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while (!imuPath.poses.empty() && stamp2Sec(imuPath.poses.front().header.stamp) < lidarOdomTime - 1.0) imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath->get_subscription_count() != 0) {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath->publish(imuPath);
            }
        }
    }
};

class IMUPreintegration : public ParamServer {
public:
    std::mutex mtx;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
    rclcpp::Subscription<lvi_sam_msgs::msg::LocalizationReset>::SharedPtr
        subLocalizationReset;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
    rclcpp::Publisher<lvi_sam_msgs::msg::LocalizationReset>::SharedPtr
        pubLocalizationReset;
    // rclcpp::Publisher<autorccar_interfaces::msg::NavState>::SharedPtr pubNavState;

    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    std::unique_ptr<gtsam::PreintegratedImuMeasurements> imuIntegratorOpt_;
    std::unique_ptr<gtsam::PreintegratedImuMeasurements> imuIntegratorImu_;

    std::deque<sensor_msgs::msg::Imu> imuQueOpt;
    std::deque<sensor_msgs::msg::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_received = -1;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;
    int imuPreintegrationResetId = 0;
    double lastCorrectionTime = -1.0;
    std::uint64_t localizationResetEventSequence = 0;
    std::string lastResetSource;
    std::uint64_t lastResetEventId = 0;
    bool hasLastResetEvent = false;

    // imuToLidarTranslation is the lever arm from the physical IMU origin to the
    // LiDAR origin, expressed in the raw IMU axes (the same convention as the
    // URDF-derived IMU -> LiDAR translation and FAST-LIO's extrinsic_T).
    //
    // imuConverter() rotates IMU acceleration and angular velocity into the
    // LiDAR axes with imuToLidarRotation, so the preintegrated "IMU pose" uses a virtual
    // IMU frame whose axes are parallel to the LiDAR frame.  Rotate the lever
    // arm into those axes before composing poses, then obtain the reverse
    // transform by inversion instead of relying on an ambiguous sign.
    Eigen::Vector3d imuToLidarTrans =
        imuToLidarRotation * imuToLidarTranslation;
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(
        gtsam::Rot3(1, 0, 0, 0),
        gtsam::Point3(
            imuToLidarTrans.x(),
            imuToLidarTrans.y(),
            imuToLidarTrans.z()));
    gtsam::Pose3 lidar2Imu = imu2Lidar.inverse();

    explicit IMUPreintegration(const rclcpp::NodeOptions& options)
        : ParamServer("IMUPreintegrationParamServer", options) {
        callbackGroupImu = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupOdom = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto imuOpt = rclcpp::SubscriptionOptions();
        imuOpt.callback_group = callbackGroupImu;
        auto odomOpt = rclcpp::SubscriptionOptions();
        odomOpt.callback_group = callbackGroupOdom;

        subImu = create_subscription<sensor_msgs::msg::Imu>(imuTopic, qos_imu, std::bind(&IMUPreintegration::imuHandler, this, std::placeholders::_1), imuOpt);
        subOdometry = create_subscription<nav_msgs::msg::Odometry>("lio_sam/mapping/odometry_incremental", qos,
                                                                   std::bind(&IMUPreintegration::odometryHandler, this, std::placeholders::_1), odomOpt);
        subLocalizationReset = create_subscription<lvi_sam_msgs::msg::LocalizationReset>(
            localizationResetTopic, rclcpp::QoS(10).reliable(),
            std::bind(&IMUPreintegration::localizationResetHandler, this,
                      std::placeholders::_1), odomOpt);

        pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic + "_incremental", qos_imu);
        pubLocalizationReset = create_publisher<lvi_sam_msgs::msg::LocalizationReset>(
            localizationResetTopic, rclcpp::QoS(10).reliable());
        // pubNavState = create_publisher<autorccar_interfaces::msg::NavState>("/nav_topic", 10);

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2);  // acc white noise in continuous
        p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2);      // gyro white noise in continuous
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);           // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());  // assume zero initial bias

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());  // rad,rad,rad,m, m, m
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                                // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);  // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());  // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());                // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        imuIntegratorImu_ =
            std::make_unique<gtsam::PreintegratedImuMeasurements>(
                p, prior_imu_bias);
        imuIntegratorOpt_ =
            std::make_unique<gtsam::PreintegratedImuMeasurements>(
                p, prior_imu_bias);
    }

    void resetOptimization() {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams() {
        imuQueOpt.clear();
        imuQueImu.clear();
        lastImuT_received = -1;
        lastImuT_imu = -1;
        lastImuT_opt = -1;
        lastCorrectionTime = -1.0;
        doneFirstOpt = false;
        systemInitialized = false;
        key = 1;
        resetOptimization();
        const gtsam::imuBias::ConstantBias zeroBias;
        if (imuIntegratorOpt_) {
            imuIntegratorOpt_->resetIntegrationAndSetBias(zeroBias);
        }
        if (imuIntegratorImu_) {
            imuIntegratorImu_->resetIntegrationAndSetBias(zeroBias);
        }
    }

    void publishImuFailureResetEvent(const std::string& detail) {
        ++localizationResetEventSequence;
        if (!pubLocalizationReset) return;
        lvi_sam_msgs::msg::LocalizationReset msg;
        msg.header.stamp = this->now().to_msg();
        msg.header.frame_id = odometryFrame;
        msg.event_id = localizationResetEventSequence;
        msg.reset_id = static_cast<std::uint64_t>(
            std::max(0, imuPreintegrationResetId));
        msg.reason = lvi_sam_msgs::msg::LocalizationReset::REASON_IMU_FAILURE;
        msg.source = "imu_preintegration";
        msg.reset_imu = true;
        msg.restart_visual = true;
        msg.detail = detail;
        pubLocalizationReset->publish(msg);
        RCLCPP_WARN(
            get_logger(),
            "Published IMU failure reset event: reset_id=%llu event_id=%llu detail=%s",
            static_cast<unsigned long long>(msg.reset_id),
            static_cast<unsigned long long>(msg.event_id), detail.c_str());
    }

    void localizationResetHandler(
        const lvi_sam_msgs::msg::LocalizationReset::SharedPtr msg) {
        if (!msg) return;
        std::lock_guard<std::mutex> lock(mtx);
        if (hasLastResetEvent && lastResetSource == msg->source &&
            lastResetEventId == msg->event_id) {
            return;
        }
        lastResetSource = msg->source;
        lastResetEventId = msg->event_id;
        hasLastResetEvent = true;
        if (!msg->reset_imu) return;

        // This node publishes IMU_FAILURE events on the same topic.  Its
        // integration state has already been cleared before publishing, so
        // applying the event again would unnecessarily discard samples that
        // arrived in the meantime.  TransformFusion still handles the event
        // because it is a separate consumer with its own propagation queue.
        if (msg->source == "imu_preintegration") return;

        // If the compatibility covariance channel already carried this
        // map-owned generation, the state has already been reset. This guard
        // prevents a late event from discarding new IMU samples.
        if (msg->source == "map_optimization" &&
            msg->reset_id == static_cast<std::uint64_t>(
                std::max(0, imuPreintegrationResetId))) {
            return;
        }
        if (msg->source == "map_optimization" &&
            msg->reset_id > static_cast<std::uint64_t>(
                std::numeric_limits<int>::max())) {
            RCLCPP_ERROR(
                get_logger(),
                "Ignoring map reset_id=%llu because the legacy covariance metadata is int-sized",
                static_cast<unsigned long long>(msg->reset_id));
            return;
        }

        resetParams();
        if (msg->source == "map_optimization") {
            imuPreintegrationResetId = static_cast<int>(msg->reset_id);
        }
        RCLCPP_INFO(
            get_logger(),
            "Applied localization reset event: source=%s reason=%u reset_id=%llu detail=%s",
            msg->source.c_str(), static_cast<unsigned int>(msg->reason),
            static_cast<unsigned long long>(msg->reset_id), msg->detail.c_str());
    }

    bool integrateImuMeasurement(
        gtsam::PreintegratedImuMeasurements* integrator,
        const sensor_msgs::msg::Imu& imu,
        double& lastImuTime,
        const char* stage) {
        const double imuTime = stamp2Sec(imu.header.stamp);
        const double dt = (lastImuTime < 0) ? (1.0 / 500.0) : (imuTime - lastImuTime);

        if (!std::isfinite(imuTime) || !std::isfinite(dt) || dt <= 0.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Discarding IMU sample during %s: timestamp=%.9f, previous=%.9f, dt=%.9f",
                stage,
                imuTime,
                lastImuTime,
                dt);
            return false;
        }

        integrator->integrateMeasurement(
            gtsam::Vector3(
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z),
            gtsam::Vector3(
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z),
            dt);
        lastImuTime = imuTime;
        return true;
    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        std::lock_guard<std::mutex> lock(mtx);

        const double currentCorrectionTime = stamp2Sec(odomMsg->header.stamp);
        const double resetIdValue = odomMsg->pose.covariance[
            lvi_sam::internal_odom_metadata::mapping_correction::kResetId];
        const double degenerateValue = odomMsg->pose.covariance[
            lvi_sam::internal_odom_metadata::mapping_correction::kDegenerate];
        const auto& position = odomMsg->pose.pose.position;
        const auto& orientation = odomMsg->pose.pose.orientation;
        const double quaternionNorm = std::sqrt(
            orientation.x * orientation.x + orientation.y * orientation.y +
            orientation.z * orientation.z + orientation.w * orientation.w);
        if (!std::isfinite(currentCorrectionTime) ||
            (lastCorrectionTime >= 0.0 && currentCorrectionTime <= lastCorrectionTime) ||
            !std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
            !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
            !std::isfinite(orientation.w) || quaternionNorm < 1e-9 ||
            !std::isfinite(resetIdValue) || resetIdValue < 0.0 ||
            std::abs(resetIdValue - std::round(resetIdValue)) > 1e-6 ||
            !std::isfinite(degenerateValue)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Discarding invalid or non-monotonic mapping correction at %.9f",
                currentCorrectionTime);
            return;
        }
        lastCorrectionTime = currentCorrectionTime;

        const int currentResetId = static_cast<int>(std::llround(resetIdValue));
        if (currentResetId != imuPreintegrationResetId) {
            resetParams();
            imuPreintegrationResetId = currentResetId;
            return;
        }

        // make sure we have imu data to integrate
        if (imuQueOpt.empty()) return;

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        const bool degenerate = degenerateValue >= 0.5;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

        // 0. initialize system
        if (systemInitialized == false) {
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty()) {
                if (stamp2Sec(imuQueOpt.front().header.stamp) < currentCorrectionTime - delta_t) {
                    lastImuT_opt = stamp2Sec(imuQueOpt.front().header.stamp);
                    imuQueOpt.pop_front();
                } else
                    break;
            }

            // gtsam::Rot3 newRotation = gtsam::Rot3::RzRyRx(0.0, DEG2RAD(65.8), 0.0);
            // lidarPose = gtsam::Pose3(newRotation, lidarPose.translation());
            std::cout << "initial pose: " << lidarPose << std::endl;

            // initial pose
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            key = 1;
            systemInitialized = true;
            return;
        }

        // reset graph for speed
        if (key == 100) {
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }

        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty()) {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::msg::Imu* thisImu = &imuQueOpt.front();
            double imuTime = stamp2Sec(thisImu->header.stamp);
            if (imuTime < currentCorrectionTime - delta_t) {
                integrateImuMeasurement(
                    imuIntegratorOpt_.get(),
                    *thisImu,
                    lastImuT_opt,
                    "optimization");
                imuQueOpt.pop_front();
            } else
                break;
        }
        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu =
            *imuIntegratorOpt_;
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
            B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
            gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // add pose factor
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        // optimize
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_ = result.at<gtsam::Pose3>(X(key));
        prevVel_ = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        if (failureDetection(prevVel_, prevBias_)) {
            resetParams();
            publishImuFailureResetEvent("optimized_velocity_or_bias_threshold");
            return;
        }

        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && stamp2Sec(imuQueImu.front().header.stamp) < currentCorrectionTime - delta_t) {
            lastImuQT = stamp2Sec(imuQueImu.front().header.stamp);
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty()) {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i) {
                sensor_msgs::msg::Imu* thisImu = &imuQueImu[i];
                integrateImuMeasurement(
                    imuIntegratorImu_.get(),
                    *thisImu,
                    lastImuQT,
                    "repropagation");
            }
            lastImuT_imu = lastImuQT;
        }

        ++key;
        doneFirstOpt = true;
    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30) {
            RCLCPP_WARN(get_logger(), "Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0) {
            RCLCPP_WARN(get_logger(), "Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw) {
        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw);
        const double imuTime = stamp2Sec(thisImu.header.stamp);
        const auto& acceleration = thisImu.linear_acceleration;
        const auto& angularVelocity = thisImu.angular_velocity;

        if (!std::isfinite(imuTime) ||
            !std::isfinite(acceleration.x) ||
            !std::isfinite(acceleration.y) ||
            !std::isfinite(acceleration.z) ||
            !std::isfinite(angularVelocity.x) ||
            !std::isfinite(angularVelocity.y) ||
            !std::isfinite(angularVelocity.z) ||
            (lastImuT_received >= 0 && imuTime <= lastImuT_received)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Discarding invalid or non-monotonic IMU sample: timestamp=%.9f, previous=%.9f",
                imuTime,
                lastImuT_received);
            return;
        }
        lastImuT_received = imuTime;

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);
        while (imuQueOpt.size() > 10000) imuQueOpt.pop_front();
        while (imuQueImu.size() > 10000) imuQueImu.pop_front();

        if (doneFirstOpt == false) return;

        // integrate this single imu message
        if (!integrateImuMeasurement(
                imuIntegratorImu_.get(),
                thisImu,
                lastImuT_imu,
                "real-time propagation")) {
            return;
        }

        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        auto odometry = nav_msgs::msg::Odometry();
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = lidarFrame;

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        // Compatibility metadata for the tightly coupled visual initializer.
        // TransformFusion preserves these fields on odomTopic. They are an
        // internal contract, not a statistical pose covariance matrix.
        odometry.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kResetId] =
            static_cast<double>(imuPreintegrationResetId);
        odometry.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kAccelBiasX] =
            prevBiasOdom.accelerometer().x();
        odometry.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kAccelBiasY] =
            prevBiasOdom.accelerometer().y();
        odometry.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kAccelBiasZ] =
            prevBiasOdom.accelerometer().z();
        odometry.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kGyroBiasX] =
            prevBiasOdom.gyroscope().x();
        odometry.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kGyroBiasY] =
            prevBiasOdom.gyroscope().y();
        odometry.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kGyroBiasZ] =
            prevBiasOdom.gyroscope().z();
        odometry.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kGravity] =
            imuGravity;
        pubImuOdometry->publish(odometry);

        // autorccar_interfaces::msg::NavState navState;
        // navState.timestamp = odometry.header.stamp;
        // navState.position.x = odometry.pose.pose.position.y;
        // navState.position.y = odometry.pose.pose.position.x;
        // navState.position.z = -odometry.pose.pose.position.z;
        // navState.velocity.x = odometry.twist.twist.linear.y;
        // navState.velocity.y = odometry.twist.twist.linear.x;
        // navState.velocity.z = -odometry.twist.twist.linear.z;
        // navState.quaternion.w = -0.7071 * odometry.pose.pose.orientation.x - 0.7071 * odometry.pose.pose.orientation.y;
        // navState.quaternion.x = 0.7071 * odometry.pose.pose.orientation.w - 0.7071 * odometry.pose.pose.orientation.z;
        // navState.quaternion.y = 0.7071 * odometry.pose.pose.orientation.w + 0.7071 * odometry.pose.pose.orientation.z;
        // navState.quaternion.z = 0.7071 * odometry.pose.pose.orientation.x - 0.7071 * odometry.pose.pose.orientation.y;
        // pubNavState->publish(navState);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor e;

    auto ImuP = std::make_shared<IMUPreintegration>(options);
    auto TF = std::make_shared<TransformFusion>(options);
    e.add_node(ImuP);
    e.add_node(TF);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> IMU Preintegration Started.\033[0m");

    e.spin();

    rclcpp::shutdown();
    return 0;
}
