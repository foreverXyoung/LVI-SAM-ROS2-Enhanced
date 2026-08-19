#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <lvi_sam_msgs/msg/localization_reset.hpp>

#include "estimator.h"
#include "lvi_sam/internal_odom_metadata.hpp"
#include "parameters.h"
#include "utility/visualization.h"
#include "lvi_sam/topic_names.hpp"

using std::placeholders::_1;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buf; 
queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> feature_buf;

constexpr std::size_t kMaxImuQueueSize = 10000;
constexpr std::size_t kMaxFeatureQueueSize = 200;
constexpr std::size_t kMaxOdomQueueSize = 10000;

// global variable saving the lidar odometry
deque<nav_msgs::msg::Odometry> odomQueue;
std::unique_ptr<OdometryRegister> odomRegister;

std::mutex m_buf;
std::mutex m_state;
std::mutex m_estimator;
std::mutex m_odom;
std::mutex m_reset;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;
double last_feature_t = -1;
double last_odom_t = -1;
std::atomic<std::uint64_t> reset_generation{0};
std::uint64_t visualResetEventSequence = 0;
std::uint64_t latestLidarResetId = 0;
rclcpp::Publisher<lvi_sam_msgs::msg::LocalizationReset>::SharedPtr
    pubLocalizationReset;

using FeatureObservations =
    map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>>;

bool decodeFeatureMessage(
    const sensor_msgs::msg::PointCloud::ConstSharedPtr &feature_msg,
    FeatureObservations &image)
{
    if (!std::isfinite(rclcpp::Time(feature_msg->header.stamp).seconds()) ||
        feature_msg->channels.size() < 6)
        return false;

    for (std::size_t channel_index = 0; channel_index < 6; ++channel_index)
    {
        if (feature_msg->channels[channel_index].values.size() <
            feature_msg->points.size())
            return false;
    }

    for (std::size_t i = 0; i < feature_msg->points.size(); ++i)
    {
        const double encoded_id = feature_msg->channels[0].values[i];
        if (!std::isfinite(encoded_id) || encoded_id < 0.0 ||
            encoded_id > static_cast<double>(std::numeric_limits<int>::max()) - 0.5)
            return false;

        const int combined_id = static_cast<int>(encoded_id + 0.5);
        const int feature_id = combined_id / NUM_OF_CAM;
        const int camera_id = combined_id % NUM_OF_CAM;
        const double x = feature_msg->points[i].x;
        const double y = feature_msg->points[i].y;
        const double z = feature_msg->points[i].z;
        const double p_u = feature_msg->channels[1].values[i];
        const double p_v = feature_msg->channels[2].values[i];
        const double velocity_x = feature_msg->channels[3].values[i];
        const double velocity_y = feature_msg->channels[4].values[i];
        const double depth = feature_msg->channels[5].values[i];

        if (combined_id < 0 || feature_id < 0 || camera_id < 0 ||
            camera_id >= NUM_OF_CAM || !std::isfinite(x) ||
            !std::isfinite(y) || !std::isfinite(z) ||
            std::abs(z - 1.0) > 1e-6 || !std::isfinite(p_u) ||
            !std::isfinite(p_v) || !std::isfinite(velocity_x) ||
            !std::isfinite(velocity_y) || !std::isfinite(depth))
            return false;

        Eigen::Matrix<double, 8, 1> observation;
        observation << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
        image[feature_id].emplace_back(camera_id, observation);
    }
    return true;
}

bool validImuSequence(
    const std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> &imu_messages,
    double image_time,
    double integration_time)
{
    for (const auto &imu_msg : imu_messages)
    {
        const double imu_time = rclcpp::Time(imu_msg->header.stamp).seconds();
        if (!std::isfinite(imu_time))
            return false;

        if (imu_time <= image_time)
        {
            if (integration_time < 0.0)
                integration_time = imu_time;
            const double dt = imu_time - integration_time;
            if (!std::isfinite(dt) || dt < 0.0)
                return false;
            integration_time = imu_time;
        }
        else
        {
            const double dt_before_image = image_time - integration_time;
            const double dt_after_image = imu_time - image_time;
            if (!std::isfinite(dt_before_image) ||
                !std::isfinite(dt_after_image) || dt_before_image < 0.0 ||
                dt_after_image < 0.0 ||
                dt_before_image + dt_after_image <= 0.0)
                return false;
            integration_time = image_time;
        }
    }
    return true;
}

void predict(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg, Estimator &estimator)
{
    double t = rclcpp::Time(imu_msg->header.stamp).seconds();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    if (!std::isfinite(t) || !std::isfinite(dt) || dt <= 0.0)
        return;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update(Estimator &estimator)
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::msg::Imu::ConstSharedPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::msg::Imu::ConstSharedPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front(), estimator);
}

std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>>
getMeasurements(const double time_offset)
{
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>> measurements;

    while (rclcpp::ok())
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(rclcpp::Time(imu_buf.back()->header.stamp).seconds() > rclcpp::Time(feature_buf.front()->header.stamp).seconds() + time_offset))
        {
            return measurements;
        }

        if (!(rclcpp::Time(imu_buf.front()->header.stamp).seconds() < rclcpp::Time(feature_buf.front()->header.stamp).seconds() + time_offset))
        {
            // RCLPCPP_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::msg::PointCloud::ConstSharedPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> IMUs;
        while (rclcpp::Time(imu_buf.front()->header.stamp).seconds() < rclcpp::Time(img_msg->header.stamp).seconds() + time_offset)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        // if (IMUs.empty())
        //     ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg, Estimator &estimator)
{
    const double imu_time = rclcpp::Time(imu_msg->header.stamp).seconds();
    const auto &acceleration = imu_msg->linear_acceleration;
    const auto &angular_velocity = imu_msg->angular_velocity;
    if (!std::isfinite(imu_time) ||
        !std::isfinite(acceleration.x) || !std::isfinite(acceleration.y) ||
        !std::isfinite(acceleration.z) || !std::isfinite(angular_velocity.x) ||
        !std::isfinite(angular_velocity.y) || !std::isfinite(angular_velocity.z))
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_estimator"),
                    "Discarding IMU sample with a non-finite timestamp or measurement");
        return;
    }
    if (imu_time <= last_imu_t)
    {
        // ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_time;

    // Livox MID-360 publishes acceleration in g. Normalize the selected IMU
    // profile to sensor_msgs SI units before the message enters either the
    // buffered estimator path or the real-time prediction path.
    auto normalized_imu = std::make_shared<sensor_msgs::msg::Imu>(*imu_msg);
    normalized_imu->linear_acceleration.x *= IMU_ACCELERATION_SCALE;
    normalized_imu->linear_acceleration.y *= IMU_ACCELERATION_SCALE;
    normalized_imu->linear_acceleration.z *= IMU_ACCELERATION_SCALE;

    {
        std::lock_guard<std::mutex> lock(m_buf);
        imu_buf.push(normalized_imu);
        while (imu_buf.size() > kMaxImuQueueSize)
            imu_buf.pop();
    }
    con.notify_one();

    {
        std::scoped_lock lock(m_estimator, m_state);
        predict(normalized_imu, estimator);
        std_msgs::msg::Header header = normalized_imu->header;
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header, estimator.failureCount);
    }
}

void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) 
{
    const auto &position = odom_msg->pose.pose.position;
    const auto &orientation = odom_msg->pose.pose.orientation;
    const auto &velocity = odom_msg->twist.twist.linear;
    const double quaternion_norm = std::sqrt(
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w);
    bool covariance_is_finite = true;
    for (std::size_t i = 0; i < 8; ++i)
        covariance_is_finite = covariance_is_finite &&
                               std::isfinite(odom_msg->pose.covariance[i]);
    const double reset_id_value = odom_msg->pose.covariance[
        lvi_sam::internal_odom_metadata::visual_prior::kResetId];
    if (!std::isfinite(rclcpp::Time(odom_msg->header.stamp).seconds()) ||
        !std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
        !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
        !std::isfinite(orientation.w) || quaternion_norm < 1e-9 ||
        !std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
        !std::isfinite(velocity.z) || !covariance_is_finite ||
        !std::isfinite(reset_id_value) || reset_id_value < 0.0 ||
        std::abs(reset_id_value - std::round(reset_id_value)) > 1e-6)
        return;
    const std::uint64_t reset_id = static_cast<std::uint64_t>(
        std::llround(reset_id_value));
    {
        std::lock_guard<std::mutex> lock(m_reset);
        // DDS does not order messages across the reset and odometry topics.
        // Once a map generation is known, reject late odometry from an older
        // generation instead of reintroducing it after the queue was cleared.
        if (reset_id < latestLidarResetId)
            return;
        if (reset_id > latestLidarResetId)
            latestLidarResetId = reset_id;
    }
    std::lock_guard<std::mutex> lock(m_odom);
    const double odom_time = rclcpp::Time(odom_msg->header.stamp).seconds();
    if (last_odom_t >= 0.0 && odom_time <= last_odom_t)
        return;
    last_odom_t = odom_time;
    odomQueue.push_back(*odom_msg);
    while (odomQueue.size() > kMaxOdomQueueSize)
        odomQueue.pop_front();
}

void feature_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &feature_msg)
{
    const double feature_time =
        rclcpp::Time(feature_msg->header.stamp).seconds();
    if (!std::isfinite(feature_time))
        return;
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    {
        std::lock_guard<std::mutex> lock(m_buf);
        if (last_feature_t >= 0.0 && feature_time <= last_feature_t)
            return;
        last_feature_t = feature_time;
        feature_buf.push(feature_msg);
        while (feature_buf.size() > kMaxFeatureQueueSize)
            feature_buf.pop();
    }
    con.notify_one();
}

void resetEstimatorState(Estimator &estimator)
{
    std::scoped_lock lock(m_buf, m_state, m_estimator, m_odom);
    while (!feature_buf.empty())
        feature_buf.pop();
    while (!imu_buf.empty())
        imu_buf.pop();
    estimator.clearState();
    estimator.setParameter();
    current_time = -1;
    latest_time = 0;
    last_imu_t = 0;
    last_feature_t = -1;
    init_imu = true;
    init_feature = false;
    tmp_P.setZero();
    tmp_Q.setIdentity();
    tmp_V.setZero();
    tmp_Ba.setZero();
    tmp_Bg.setZero();
    acc_0.setZero();
    gyr_0.setZero();
    reset_generation.fetch_add(1, std::memory_order_release);
    odomQueue.clear();
    last_odom_t = -1;
    resetVisualTfState();
}

// Estimator::processImage() clears its own sliding-window state when VINS
// failure detection fires.  This companion path clears only the ROS-side
// queues and prediction/visualization anchors; it deliberately does not call
// estimator.clearState() again while the process thread already owns
// m_estimator.
void resetEstimatorAuxiliaryStateAfterFailure()
{
    std::scoped_lock lock(m_buf, m_state, m_odom);
    while (!feature_buf.empty())
        feature_buf.pop();
    while (!imu_buf.empty())
        imu_buf.pop();
    current_time = -1;
    latest_time = 0;
    last_imu_t = 0;
    last_feature_t = -1;
    init_imu = true;
    init_feature = false;
    tmp_P.setZero();
    tmp_Q.setIdentity();
    tmp_V.setZero();
    tmp_Ba.setZero();
    tmp_Bg.setZero();
    acc_0.setZero();
    gyr_0.setZero();
    reset_generation.fetch_add(1, std::memory_order_release);
    odomQueue.clear();
    // Keep last_odom_t as a timestamp barrier.  This rejects delayed old
    // odometry from the same map generation without rejecting a valid stream
    // whose timestamps remain monotonic after the VINS-only restart.
    resetVisualTfState();
}

void restart_callback(
    const std_msgs::msg::Bool::ConstSharedPtr &restart_msg,
    Estimator &estimator)
{
    if (restart_msg && restart_msg->data)
    {
        resetEstimatorState(estimator);
        con.notify_all();
    }
}

void publishVisualResetEvent(
    const std_msgs::msg::Header &header,
    const std::uint64_t reset_id,
    const std::string &detail)
{
    if (!pubLocalizationReset)
        return;
    ++visualResetEventSequence;
    lvi_sam_msgs::msg::LocalizationReset reset_msg;
    reset_msg.header = header;
    reset_msg.event_id = visualResetEventSequence;
    reset_msg.reset_id = reset_id;
    reset_msg.reason = lvi_sam_msgs::msg::LocalizationReset::REASON_VINS_FAILURE;
    reset_msg.source = "visual_estimator";
    reset_msg.reset_imu = false;
    reset_msg.restart_visual = true;
    reset_msg.detail = detail;
    pubLocalizationReset->publish(reset_msg);
}

// thread: visual-inertial odometry
void process(Estimator &estimator)
{
    while (rclcpp::ok())
    {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud::ConstSharedPtr>> measurements;
        std::uint64_t measurement_generation = 0;
        bool auxiliary_reset_requested = false;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            if (!rclcpp::ok())
                return true;
            double time_offset = 0.0;
            {
                std::lock_guard<std::mutex> estimator_lock(m_estimator);
                time_offset = estimator.td;
            }
            measurements = getMeasurements(time_offset);
            measurement_generation =
                reset_generation.load(std::memory_order_acquire);
            return !measurements.empty();
                 });
        if (!rclcpp::ok())
            break;
        lk.unlock();

        {
        std::lock_guard<std::mutex> estimator_lock(m_estimator);
        if (measurement_generation !=
            reset_generation.load(std::memory_order_acquire))
            continue;
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;

            FeatureObservations image;
            if (!decodeFeatureMessage(img_msg, image))
            {
                RCLCPP_WARN(rclcpp::get_logger("visual_estimator"),
                            "Discarding malformed feature cloud");
                continue;
            }

            const double image_time =
                rclcpp::Time(img_msg->header.stamp).seconds() + estimator.td;
            if (!validImuSequence(measurement.first, image_time, current_time))
            {
                RCLCPP_WARN(rclcpp::get_logger("visual_estimator"),
                            "Discarding feature frame with an invalid IMU time sequence");
                continue;
            }

            // 1. IMU pre-integration
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = rclcpp::Time(imu_msg->header.stamp).seconds();
                if (t <= image_time)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else
                {
                    double dt_1 = image_time - current_time;
                    double dt_2 = t - image_time;
                    current_time = image_time;
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // 2. VINS Optimization
            // TicToc t_s;
            // Get initialization info from lidar odometry
            vector<float> initialization_info;
            {
                std::lock_guard<std::mutex> lock(m_odom);
                initialization_info = odomRegister->getOdometry(
                    odomQueue,
                    rclcpp::Time(img_msg->header.stamp).seconds() + estimator.td);
            }

            if (!initialization_info.empty() &&
                std::isfinite(initialization_info[0]) &&
                initialization_info[0] >= 0.0f &&
                std::abs(initialization_info[0] -
                         std::round(initialization_info[0])) < 1e-6f) {
                std::lock_guard<std::mutex> lock(m_reset);
                latestLidarResetId = static_cast<std::uint64_t>(
                    std::llround(initialization_info[0]));
            }


            estimator.processImage(image, initialization_info, img_msg->header);
            if (estimator.failure_event_pending) {
                std::uint64_t reset_id = 0;
                {
                    std::lock_guard<std::mutex> lock(m_reset);
                    reset_id = latestLidarResetId;
                }
                publishVisualResetEvent(
                    img_msg->header, reset_id,
                    "vins_failure_detection");
                auxiliary_reset_requested = true;
                // Keep the original estimator flags untouched; consume only
                // the new one-shot notification bit.
                estimator.failure_event_pending = false;
                break;
            }
            // double whole_t = t_s.toc();
            // printStatistics(estimator, whole_t);

            // 3. Visualization
            std_msgs::msg::Header header = img_msg->header;
            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
        }
        }

        if (auxiliary_reset_requested)
            resetEstimatorAuxiliaryStateAfterFailure();

        std::scoped_lock state_lock(m_buf, m_state, m_estimator);
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update(estimator);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vins");
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator Started.\033[0m");
    // if (rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN) != RCUTILS_RET_OK) {
    //     RCLCPP_WARN(node->get_logger(), "Failed to set logger level.");
    // }
    Estimator estimator;

    readParameters(node);
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator readParameters completed.\033[0m");
    estimator.setParameter();
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator setParameter completed.\033[0m");
    registerPub(node);
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator registerPub completed.\033[0m");
    pubLocalizationReset =
        node->create_publisher<lvi_sam_msgs::msg::LocalizationReset>(
            LOCALIZATION_RESET_TOPIC, rclcpp::QoS(10).reliable());
    odomRegister = std::make_unique<OdometryRegister>(node);
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator odometryRegister constructor completed.\033[0m");

    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
        IMU_TOPIC, rclcpp::SensorDataQoS().keep_last(5000),
        std::function<void(const sensor_msgs::msg::Imu::ConstSharedPtr&)>(
            std::bind(imu_callback, std::placeholders::_1, std::ref(estimator))));

    auto sub_odom = node->create_subscription<nav_msgs::msg::Odometry>(
        ODOM_TOPIC, rclcpp::SensorDataQoS().keep_last(5000),
        odom_callback);
    if (!USE_LIDAR_ODOMETRY_PRIOR)
        sub_odom.reset();

    auto sub_image = node->create_subscription<sensor_msgs::msg::PointCloud>(
        lvi_sam::topics::project_topic(PROJECT_NAME, lvi_sam::topics::kFeature), 1,
        feature_callback);

    auto sub_restart = node->create_subscription<std_msgs::msg::Bool>(
        lvi_sam::topics::project_topic(PROJECT_NAME, lvi_sam::topics::kFeatureRestart), 1,
        std::function<void(const std_msgs::msg::Bool::ConstSharedPtr&)>(
            std::bind(restart_callback, std::placeholders::_1, std::ref(estimator))));

    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator Subscribers created.\033[0m");

    std::thread measurement_process{process, std::ref(estimator)};
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator measurement_process created.\033[0m");

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator executor created.\033[0m");
    executor.add_node(node);
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator add_node done.\033[0m");
    executor.spin();
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Odometry Estimator spin done.\033[0m");

    rclcpp::shutdown();
    con.notify_all();

    if (measurement_process.joinable())
        measurement_process.join();

    return 0;
}
