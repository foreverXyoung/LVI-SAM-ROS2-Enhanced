#include "lvi_sam/visual_frame_conventions.hpp"

#include <gtest/gtest.h>

namespace
{

constexpr double kTolerance = 1e-12;

TEST(VisualFrameConventions, FixedRotationsAreProper)
{
    const auto camera_from_depth =
        lvi_sam::visual_frames::vins_camera_from_depth_frame();
    const auto world_from_odom =
        lvi_sam::visual_frames::vins_world_from_ros_odom();

    EXPECT_NEAR(camera_from_depth.linear().determinant(), 1.0, kTolerance);
    EXPECT_NEAR(world_from_odom.linear().determinant(), 1.0, kTolerance);
    EXPECT_TRUE(camera_from_depth.linear().isUnitary(kTolerance));
    EXPECT_TRUE(world_from_odom.linear().isUnitary(kTolerance));
}

TEST(VisualFrameConventions, CalibrationRoundTripsPoints)
{
    const auto depth_from_lidar =
        lvi_sam::visual_frames::depth_frame_from_lidar(
            0.05, -0.07, -0.07, 0.01, -0.02, -0.04);
    const Eigen::Vector3d lidar_point(2.0, -0.4, 0.8);

    const Eigen::Vector3d recovered =
        depth_from_lidar.inverse() * (depth_from_lidar * lidar_point);
    EXPECT_TRUE(recovered.isApprox(lidar_point, kTolerance));
}

TEST(VisualFrameConventions, ZeroCalibrationMatchesUpstreamAxes)
{
    const auto camera_from_lidar =
        lvi_sam::visual_frames::vins_camera_from_depth_frame() *
        lvi_sam::visual_frames::depth_frame_from_lidar(0, 0, 0, 0, 0, 0);
    const auto world_from_camera =
        lvi_sam::visual_frames::vins_world_from_ros_odom() *
        Eigen::Affine3d::Identity() * camera_from_lidar.inverse();

    const Eigen::Matrix3d expected =
        Eigen::AngleAxisd(
            lvi_sam::visual_frames::kPi, Eigen::Vector3d::UnitZ())
            .toRotationMatrix() *
        Eigen::AngleAxisd(
            lvi_sam::visual_frames::kPi, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
    EXPECT_TRUE(world_from_camera.linear().isApprox(expected, kTolerance));
    EXPECT_TRUE(world_from_camera.translation().isZero(kTolerance));
}

}  // namespace
