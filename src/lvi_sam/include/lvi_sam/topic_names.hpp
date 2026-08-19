#pragma once

#include <cctype>
#include <stdexcept>
#include <string>
#include <string_view>

namespace lvi_sam::topics
{

inline std::string project_topic(
    const std::string &project_name, const std::string_view suffix)
{
    const auto first = project_name.find_first_not_of('/');
    const auto last = project_name.find_last_not_of('/');
    if (first == std::string::npos)
        throw std::invalid_argument("project_name must not be empty");
    if (suffix.empty() || suffix.front() != '/')
        throw std::invalid_argument("topic suffix must start with '/'");

    const std::string root = project_name.substr(first, last - first + 1);
    for (const unsigned char character : root)
    {
        if (!std::isalnum(character) && character != '_' && character != '/')
            throw std::invalid_argument(
                "project_name may contain only letters, digits, '_' and '/'");
    }
    if (root.find("//") != std::string::npos)
        throw std::invalid_argument(
            "project_name must not contain empty namespace segments");
    std::size_t segment_start = 0;
    while (segment_start < root.size())
    {
        if (std::isdigit(static_cast<unsigned char>(root[segment_start])))
            throw std::invalid_argument(
                "project_name namespace segments must not start with a digit");
        const auto separator = root.find('/', segment_start);
        if (separator == std::string::npos)
            break;
        segment_start = separator + 1;
    }

    return "/" + root + std::string(suffix);
}

inline constexpr std::string_view kFeature = "/vins/feature/feature";
inline constexpr std::string_view kFeatureImage = "/vins/feature/feature_img";
inline constexpr std::string_view kFeatureRestart = "/vins/feature/restart";
inline constexpr std::string_view kDepthFeature = "/vins/depth/depth_feature";
inline constexpr std::string_view kDepthImage = "/vins/depth/depth_image";
inline constexpr std::string_view kDepthCloud = "/vins/depth/depth_cloud";
inline constexpr std::string_view kKeyframePose = "/vins/odometry/keyframe_pose";
inline constexpr std::string_view kKeyframePoint = "/vins/odometry/keyframe_point";
inline constexpr std::string_view kExtrinsic = "/vins/odometry/extrinsic";
inline constexpr std::string_view kImuPropagate = "/vins/odometry/imu_propagate";
inline constexpr std::string_view kImuPropagateRos = "/vins/odometry/imu_propagate_ros";
inline constexpr std::string_view kPath = "/vins/odometry/path";
inline constexpr std::string_view kOdometry = "/vins/odometry/odometry";
inline constexpr std::string_view kPointCloud = "/vins/odometry/point_cloud";
inline constexpr std::string_view kHistoryCloud = "/vins/odometry/history_cloud";
inline constexpr std::string_view kKeyPoses = "/vins/odometry/key_poses";
inline constexpr std::string_view kCameraPose = "/vins/odometry/camera_pose";
inline constexpr std::string_view kCameraPoseVisual = "/vins/odometry/camera_pose_visual";
inline constexpr std::string_view kLoopMatchImage = "/vins/loop/match_image";
inline constexpr std::string_view kLoopMatchFrame = "/vins/loop/match_frame";
inline constexpr std::string_view kLoopKeyframePose = "/vins/loop/keyframe_pose";

}  // namespace lvi_sam::topics
