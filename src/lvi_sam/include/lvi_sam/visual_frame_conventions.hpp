#pragma once

#include <Eigen/Geometry>

namespace lvi_sam::visual_frames
{

inline constexpr double kPi = 3.14159265358979323846;

// The upstream visual pipeline represents the camera/body with optical-style
// axes, while depth association uses a ROS/LiDAR-style frame at the same
// origin.  Keep these fixed conventions in one place so initialization,
// visualization, and depth registration cannot silently diverge.
inline Eigen::Affine3d vins_camera_from_depth_frame()
{
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() =
        Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return transform;
}

inline Eigen::Affine3d vins_world_from_ros_odom()
{
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() =
        Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return transform;
}

// Legacy parameter names lidar_to_cam_* are retained for configuration
// compatibility. Semantically this transform maps points from the physical
// LiDAR frame into the virtual depth-projection frame (ROS/LiDAR axes at the
// VINS camera/body origin). Rotations use intrinsic X/Y/Z parameters composed
// as Rz * Ry * Rx, matching pcl::getTransformation.
inline Eigen::Affine3d depth_frame_from_lidar(
    const double tx, const double ty, const double tz,
    const double rx, const double ry, const double rz)
{
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = (
        Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()))
                             .toRotationMatrix();
    transform.translation() = Eigen::Vector3d(tx, ty, tz);
    return transform;
}

}  // namespace lvi_sam::visual_frames
