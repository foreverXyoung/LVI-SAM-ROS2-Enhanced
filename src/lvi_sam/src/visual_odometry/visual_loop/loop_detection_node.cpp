#include "parameters.h"
#include "keyframe.h"
#include "loop_detection.h"
#include "lvi_sam/image_conversion.hpp"
#include "lvi_sam/package_assets.hpp"
#include "lvi_sam/topic_names.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <utility>

#define SKIP_FIRST_CNT 10

queue<sensor_msgs::msg::Image::ConstSharedPtr>      image_buf; 
queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> point_buf;
queue<nav_msgs::msg::Odometry::ConstSharedPtr>    pose_buf;

std::mutex m_buf;
std::mutex m_process;
std::condition_variable buffer_condition;
std::uint64_t buffer_generation = 0;
std::atomic_bool reset_requested{false};

LoopDetector loopDetector;

double SKIP_TIME = 0;
double SKIP_DIST = 0;
double LOOP_SYNC_TOLERANCE = 0.02;

camodocal::CameraPtr m_camera;

Eigen::Vector3d tic = Eigen::Vector3d::Zero();
Eigen::Matrix3d qic = Eigen::Matrix3d::Identity();
bool extrinsic_received = false;

std::string PROJECT_NAME;
std::string IMAGE_TOPIC;
std::string LOCALIZATION_RESET_TOPIC;

int DEBUG_IMAGE;
int LOOP_CLOSURE;
double MATCH_IMAGE_SCALE;
int LOOP_MIN_INDEX_GAP = 200;
double LOOP_PRIMARY_SCORE_THRESHOLD = 0.05;
double LOOP_SECONDARY_SCORE_THRESHOLD = 0.015;


rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_match_msg;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_key_pose;



BriefExtractor briefExtractor;

void new_sequence()
{
    {
        std::lock_guard<std::mutex> lock(m_buf);
        while(!image_buf.empty())
            image_buf.pop();
        while(!point_buf.empty())
            point_buf.pop();
        while(!pose_buf.empty())
            pose_buf.pop();
        reset_requested.store(true);
        ++buffer_generation;
    }
    buffer_condition.notify_one();
}

void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received image data");
    // std::cout<<"Received image data in Visual Odometry\n";
    if(!LOOP_CLOSURE)
        return;

    const double image_time = rclcpp::Time(image_msg->header.stamp).seconds();
    if (!std::isfinite(image_time))
        return;

    {
        std::lock_guard<std::mutex> lock(m_buf);
        image_buf.push(image_msg);
        while (image_buf.size() > 200) image_buf.pop();
        ++buffer_generation;
    }
    buffer_condition.notify_one();

    // detect unstable camera stream
    static double last_image_time = -1;
    if (last_image_time == -1)
        last_image_time = image_time;
    else if (image_time - last_image_time > 1.0 || image_time <= last_image_time)
    {
        // ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_time;
}

void point_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &point_msg)
{
    if(!LOOP_CLOSURE)
        return;
    if (!std::isfinite(rclcpp::Time(point_msg->header.stamp).seconds()))
        return;

    {
        std::lock_guard<std::mutex> lock(m_buf);
        if (!point_buf.empty() &&
            rclcpp::Time(point_buf.back()->header.stamp).seconds() >=
                rclcpp::Time(point_msg->header.stamp).seconds())
            return;
        point_buf.push(point_msg);
        while (point_buf.size() > 200) point_buf.pop();
        ++buffer_generation;
    }
    buffer_condition.notify_one();
}

void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
    if(!LOOP_CLOSURE)
        return;
    const auto &position = pose_msg->pose.pose.position;
    const auto &orientation = pose_msg->pose.pose.orientation;
    if (!std::isfinite(rclcpp::Time(pose_msg->header.stamp).seconds()) ||
        !std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
        !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
        !std::isfinite(orientation.w))
        return;

    {
        std::lock_guard<std::mutex> lock(m_buf);
        if (!pose_buf.empty() &&
            rclcpp::Time(pose_buf.back()->header.stamp).seconds() >=
                rclcpp::Time(pose_msg->header.stamp).seconds())
            return;
        pose_buf.push(pose_msg);
        while (pose_buf.size() > 200) pose_buf.pop();
        ++buffer_generation;
    }
    buffer_condition.notify_one();
}

void extrinsic_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
    const auto &position = pose_msg->pose.pose.position;
    const auto &orientation = pose_msg->pose.pose.orientation;
    Eigen::Quaterniond rotation(
        orientation.w, orientation.x, orientation.y, orientation.z);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !rotation.coeffs().allFinite() ||
        rotation.norm() < 1e-9)
        return;
    rotation.normalize();

    std::lock_guard<std::mutex> lock(m_process);
    tic = Vector3d(position.x, position.y, position.z);
    qic = rotation.toRotationMatrix();
    extrinsic_received = true;
}

void process()
{
    if (!LOOP_CLOSURE)
        return;

    std::uint64_t observed_generation = 0;
    int skip_first_count = 0;
    double last_skip_time = -1.0;
    Eigen::Vector3d last_translation(-1e6, -1e6, -1e6);
    int global_frame_index = 0;
    while (rclcpp::ok())
    {
        sensor_msgs::msg::Image::ConstSharedPtr image_msg = nullptr;
        sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg = nullptr;
        nav_msgs::msg::Odometry::ConstSharedPtr pose_msg = nullptr;

        // Approximate-time synchronization around each estimator pose. Inputs
        // are ordered; samples that are provably too old are discarded, while
        // a pose is discarded once either available stream is already newer
        {
            std::unique_lock<std::mutex> lock(m_buf);
            buffer_condition.wait(lock, [&] {
                return !rclcpp::ok() ||
                       reset_requested.load() ||
                       buffer_generation != observed_generation;
            });
            if (!rclcpp::ok())
                break;
            observed_generation = buffer_generation;
            const std::size_t queued_before =
                image_buf.size() + point_buf.size() + pose_buf.size();
            if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
            {
                const double pose_time =
                    rclcpp::Time(pose_buf.front()->header.stamp).seconds();
                while (!image_buf.empty() &&
                       rclcpp::Time(image_buf.front()->header.stamp).seconds() <
                           pose_time - LOOP_SYNC_TOLERANCE)
                    image_buf.pop();
                while (!point_buf.empty() &&
                       rclcpp::Time(point_buf.front()->header.stamp).seconds() <
                           pose_time - LOOP_SYNC_TOLERANCE)
                    point_buf.pop();

                if (!image_buf.empty() && !point_buf.empty())
                {
                    const double image_time =
                        rclcpp::Time(image_buf.front()->header.stamp).seconds();
                    const double point_time =
                        rclcpp::Time(point_buf.front()->header.stamp).seconds();
                    if (image_time > pose_time + LOOP_SYNC_TOLERANCE ||
                        point_time > pose_time + LOOP_SYNC_TOLERANCE)
                    {
                        pose_buf.pop();
                    }
                    else
                    {
                        pose_msg = pose_buf.front();
                        pose_buf.pop();
                        image_msg = image_buf.front();
                        image_buf.pop();
                        point_msg = point_buf.front();
                        point_buf.pop();
                    }
                }
            }
            const std::size_t queued_after =
                image_buf.size() + point_buf.size() + pose_buf.size();
            if (queued_after < queued_before && observed_generation > 0)
                --observed_generation;
        }

        if (reset_requested.exchange(false))
        {
            std::lock_guard<std::mutex> lock(m_process);
            loopDetector.reset();
            skip_first_count = 0;
            last_skip_time = -1.0;
            last_translation = Eigen::Vector3d(-1e6, -1e6, -1e6);
            global_frame_index = 0;
            continue;
        }

        if (pose_msg != nullptr)
        {
            // skip fisrt few
            if (skip_first_count < SKIP_FIRST_CNT)
            {
                ++skip_first_count;
                continue;
            }

            // limit frequency
            if (rclcpp::Time(pose_msg->header.stamp).seconds() - last_skip_time < SKIP_TIME)
                continue;
            else
                last_skip_time = rclcpp::Time(pose_msg->header.stamp).seconds();

            // get keyframe pose
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Quaterniond poseQuaternion(
                pose_msg->pose.pose.orientation.w,
                pose_msg->pose.pose.orientation.x,
                pose_msg->pose.pose.orientation.y,
                pose_msg->pose.pose.orientation.z);
            if (!T.allFinite() || !std::isfinite(poseQuaternion.norm()) ||
                poseQuaternion.norm() < 1e-6)
            {
                RCLCPP_WARN(rclcpp::get_logger("visual_loop"),
                            "Discarding a non-finite visual keyframe pose");
                continue;
            }
            poseQuaternion.normalize();
            Matrix3d R = poseQuaternion.toRotationMatrix();

            // add keyframe
            if((T - last_translation).norm() > SKIP_DIST)
            {
                // convert image
                auto converted_image = lvi_sam::image_conversion::toMono8(*image_msg);
                if (!converted_image)
                {
                    static rclcpp::Clock warning_clock(RCL_STEADY_TIME);
                    RCLCPP_WARN_THROTTLE(
                        rclcpp::get_logger("visual_loop"), warning_clock, 5000,
                        "Discarding camera image: %s", converted_image.error.c_str());
                    continue;
                }

                cv::Mat image = std::move(converted_image.image);

                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                if (point_msg->channels.size() < point_msg->points.size())
                {
                    RCLCPP_WARN(rclcpp::get_logger("visual_loop"),
                                "Discarding keyframe: point/channel counts differ (%zu/%zu)",
                                point_msg->points.size(), point_msg->channels.size());
                    continue;
                }

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    if (point_msg->channels[i].values.size() < 5)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("visual_loop"),
                                    "Discarding keyframe: feature channel %u has fewer than 5 values", i);
                        point_3d.clear();
                        break;
                    }
                    const auto &source_point = point_msg->points[i];
                    bool values_are_finite =
                        std::isfinite(source_point.x) &&
                        std::isfinite(source_point.y) &&
                        std::isfinite(source_point.z);
                    for (std::size_t value_index = 0;
                         values_are_finite && value_index < 5; ++value_index)
                        values_are_finite = std::isfinite(
                            point_msg->channels[i].values[value_index]);
                    if (!values_are_finite)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("visual_loop"),
                                    "Discarding keyframe with a non-finite feature");
                        point_3d.clear();
                        break;
                    }
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);
                }
                if (point_3d.empty()) continue;

                // new keyframe
                auto keyframe = std::make_unique<KeyFrame>(
                    rclcpp::Time(pose_msg->header.stamp).seconds(),
                    global_frame_index, T, R, image, point_3d,
                    point_2d_uv, point_2d_normal, point_id);

                // detect loop
                {
                    std::lock_guard<std::mutex> lock(m_process);
                    loopDetector.addKeyFrame(
                        std::move(keyframe), extrinsic_received);
                    loopDetector.visualizeKeyPoses(
                        rclcpp::Time(pose_msg->header.stamp).seconds());
                }

                global_frame_index++;
                last_translation = T;
            }
        }

    }
} 


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // std::shared_ptr<rclcpp::Node> n;
    auto n = rclcpp::Node::make_shared("visual_loop");
    RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection Started.\033[0m");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    // Load params
    std::string config_file;
    n->declare_parameter("vins_config_file", "");
    n->get_parameter("vins_config_file", config_file);
    RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection config_file found.\033[0m");
    // cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    // if(!fsSettings.isOpened())
    // {
    //     std::cerr << "ERROR: Wrong path to settings" << std::endl;
    // }
    // Initialize global params
    n->declare_parameter("PROJECT_NAME", "");
    n->get_parameter("PROJECT_NAME", PROJECT_NAME);
    n->declare_parameter("image_topic", "");
    n->get_parameter("image_topic", IMAGE_TOPIC);
    n->declare_parameter(
        "localization_reset_topic", "/lio_sam/localization/reset");
    n->get_parameter("localization_reset_topic", LOCALIZATION_RESET_TOPIC);
    std::cout<<"Image topic in visual loop is: "<<IMAGE_TOPIC<<std::endl;
    n->declare_parameter("loop_closure", 1);
    n->get_parameter("loop_closure", LOOP_CLOSURE);
    n->declare_parameter("skip_time", 0.0);
    n->get_parameter("skip_time", SKIP_TIME);
    n->declare_parameter("skip_dist", 0.0);
    n->get_parameter("skip_dist", SKIP_DIST);
    n->declare_parameter("loop_sync_tolerance", 0.02);
    n->get_parameter("loop_sync_tolerance", LOOP_SYNC_TOLERANCE);
    n->declare_parameter("debug_image", 0);
    n->get_parameter("debug_image", DEBUG_IMAGE);
    n->declare_parameter("match_image_scale", 0.0);
    n->get_parameter("match_image_scale", MATCH_IMAGE_SCALE);
    n->declare_parameter("loop_min_index_gap", 200);
    n->get_parameter("loop_min_index_gap", LOOP_MIN_INDEX_GAP);
    n->declare_parameter("loop_primary_score_threshold", 0.05);
    n->get_parameter(
        "loop_primary_score_threshold", LOOP_PRIMARY_SCORE_THRESHOLD);
    n->declare_parameter("loop_secondary_score_threshold", 0.015);
    n->get_parameter(
        "loop_secondary_score_threshold", LOOP_SECONDARY_SCORE_THRESHOLD);
    if (PROJECT_NAME.empty() || IMAGE_TOPIC.empty())
        throw std::runtime_error("PROJECT_NAME and image_topic must not be empty");
    if (LOCALIZATION_RESET_TOPIC.empty() ||
        LOCALIZATION_RESET_TOPIC.front() != '/')
        throw std::runtime_error(
            "localization_reset_topic must be an absolute topic");
    if (LOOP_CLOSURE != 0 && LOOP_CLOSURE != 1)
        throw std::runtime_error("loop_closure must be 0 or 1");
    if (DEBUG_IMAGE != 0 && DEBUG_IMAGE != 1)
        throw std::runtime_error("debug_image must be 0 or 1");
    if (!std::isfinite(SKIP_TIME) || SKIP_TIME < 0.0 ||
        !std::isfinite(SKIP_DIST) || SKIP_DIST < 0.0)
        throw std::runtime_error("skip_time and skip_dist must be finite and non-negative");
    if (!std::isfinite(LOOP_SYNC_TOLERANCE) || LOOP_SYNC_TOLERANCE <= 0.0)
        throw std::runtime_error("loop_sync_tolerance must be finite and positive");
    if (!std::isfinite(MATCH_IMAGE_SCALE) ||
        MATCH_IMAGE_SCALE <= 0.0 || MATCH_IMAGE_SCALE > 1.0)
        throw std::runtime_error("match_image_scale must be in (0, 1]");
    if (LOOP_MIN_INDEX_GAP <= 0)
        throw std::runtime_error("loop_min_index_gap must be positive");
    if (!std::isfinite(LOOP_PRIMARY_SCORE_THRESHOLD) ||
        !std::isfinite(LOOP_SECONDARY_SCORE_THRESHOLD) ||
        LOOP_PRIMARY_SCORE_THRESHOLD <= 0.0 ||
        LOOP_PRIMARY_SCORE_THRESHOLD > 1.0 ||
        LOOP_SECONDARY_SCORE_THRESHOLD <= 0.0 ||
        LOOP_SECONDARY_SCORE_THRESHOLD > LOOP_PRIMARY_SCORE_THRESHOLD)
        throw std::runtime_error(
            "loop score thresholds must satisfy 0 < secondary <= primary <= 1");
    RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection parameters declared.\033[0m");
    // fsSettings["project_name"] >> PROJECT_NAME;  
    // fsSettings["image_topic"]  >> IMAGE_TOPIC;  
    // fsSettings["loop_closure"] >> LOOP_CLOSURE;
    // fsSettings["skip_time"]    >> SKIP_TIME;
    // fsSettings["skip_dist"]    >> SKIP_DIST;
    // fsSettings["debug_image"]  >> DEBUG_IMAGE;
    // fsSettings["match_image_scale"] >> MATCH_IMAGE_SCALE;
    
    if (LOOP_CLOSURE)
    {
        RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection LOOP_CLOSURE Started.\033[0m");
        // initialize vocabulary
        string vocabulary_file;
        n->declare_parameter("vocabulary_file", "");
        n->get_parameter("vocabulary_file", vocabulary_file);
        // fsSettings["vocabulary_file"] >> vocabulary_file;  
        vocabulary_file = lvi_sam::resolve_package_asset(
            vocabulary_file, "visual-loop vocabulary");
        loopDetector.loadVocabulary(vocabulary_file);
        RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection loadVocabulary done.\033[0m");

        // initialize brief extractor
        string brief_pattern_file;
        n->declare_parameter("brief_pattern_file", "");
        n->get_parameter("brief_pattern_file", brief_pattern_file);
        // fsSettings["brief_pattern_file"] >> brief_pattern_file;  
        brief_pattern_file = lvi_sam::resolve_package_asset(
            brief_pattern_file, "BRIEF pattern");
        briefExtractor = BriefExtractor(n, brief_pattern_file);
        RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection BriefExtractor done.\033[0m");

        // The prior ROS2 port left this disabled, but KeyFrame immediately
        // needs the camera model to normalize FAST keypoints.
        config_file = lvi_sam::resolve_package_asset(
            config_file, "camera configuration");
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(n, config_file);
        if (!m_camera)
            throw std::runtime_error("Failed to initialize the visual-loop camera model");
    }

    if (!LOOP_CLOSURE)
    {
        RCLCPP_INFO(n->get_logger(), "Visual loop closure is disabled; exiting cleanly");
        rclcpp::shutdown();
        return 0;
    }

    auto sub_image = n->create_subscription<sensor_msgs::msg::Image>(
        IMAGE_TOPIC, rclcpp::SensorDataQoS().keep_last(30),
        image_callback);

    auto sub_pose = n->create_subscription<nav_msgs::msg::Odometry>(
        lvi_sam::topics::project_topic(PROJECT_NAME, lvi_sam::topics::kKeyframePose), 3,
        pose_callback);

    auto sub_point = n->create_subscription<sensor_msgs::msg::PointCloud>(
        lvi_sam::topics::project_topic(PROJECT_NAME, lvi_sam::topics::kKeyframePoint), 3,
        point_callback);

    auto sub_extrinsic = n->create_subscription<nav_msgs::msg::Odometry>(
        lvi_sam::topics::project_topic(PROJECT_NAME, lvi_sam::topics::kExtrinsic), 3,
        extrinsic_callback);
    RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection subscribers created.\033[0m");

    pub_match_img = n->create_publisher<sensor_msgs::msg::Image>(
        lvi_sam::topics::project_topic(PROJECT_NAME, lvi_sam::topics::kLoopMatchImage), 3);
    pub_match_msg = n->create_publisher<std_msgs::msg::Float64MultiArray>(
        lvi_sam::topics::project_topic(PROJECT_NAME, lvi_sam::topics::kLoopMatchFrame), 3);
    pub_key_pose = n->create_publisher<visualization_msgs::msg::MarkerArray>(
        lvi_sam::topics::project_topic(PROJECT_NAME, lvi_sam::topics::kLoopKeyframePose), 3);
    RCLCPP_INFO(n->get_logger(), "\033[1;32m----> Visual Loop Detection publishers created.\033[0m");

    std::thread measurement_process{process};

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(n);
    executor.spin();

    rclcpp::shutdown();
    buffer_condition.notify_all();
    if (measurement_process.joinable())
        measurement_process.join();

    return 0;
}
