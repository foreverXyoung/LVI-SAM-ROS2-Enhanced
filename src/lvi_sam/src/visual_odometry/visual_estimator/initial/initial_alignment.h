#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <rclcpp/rclcpp.hpp>
#include <map>
#include "../feature_manager.h"

#include "nav_msgs/msg/odometry.hpp"
#include "lvi_sam/internal_odom_metadata.hpp"
#include "lvi_sam/visual_frame_conventions.hpp"

using namespace Eigen;
using namespace std;

class ImageFrame
{
    public:
        ImageFrame()
            : t{0.0}, pre_integration{nullptr}, is_key_frame{false},
              reset_id{-1}, T{Vector3d::Zero()}, R{Matrix3d::Identity()},
              V{Vector3d::Zero()}, Ba{Vector3d::Zero()},
              Bg{Vector3d::Zero()}, gravity{9.805} {};
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>>& _points,
                   const vector<float> &_lidar_initialization_info,
                   double _t) : ImageFrame()
        {
            points = _points;
            t = _t;

            if (_lidar_initialization_info.size() < 18)
                return;
            for (const float value : _lidar_initialization_info)
                if (!std::isfinite(value))
                    return;
            if (_lidar_initialization_info[0] < 0.0f)
                return;
            
            // reset id in case lidar odometry relocate
            reset_id = (int)round(_lidar_initialization_info[0]);
            // Pose
            T.x() = _lidar_initialization_info[1];
            T.y() = _lidar_initialization_info[2];
            T.z() = _lidar_initialization_info[3];
            // Rotation
            Eigen::Quaterniond Q = Eigen::Quaterniond(_lidar_initialization_info[7],
                                                      _lidar_initialization_info[4],
                                                      _lidar_initialization_info[5],
                                                      _lidar_initialization_info[6]);
            if (Q.norm() < 1e-9)
            {
                reset_id = -1;
                T.setZero();
                return;
            }
            R = Q.normalized().toRotationMatrix();
            // Velocity
            V.x() = _lidar_initialization_info[8];
            V.y() = _lidar_initialization_info[9];
            V.z() = _lidar_initialization_info[10];
            // Acceleration bias
            Ba.x() = _lidar_initialization_info[11];
            Ba.y() = _lidar_initialization_info[12];
            Ba.z() = _lidar_initialization_info[13];
            // Gyroscope bias
            Bg.x() = _lidar_initialization_info[14];
            Bg.y() = _lidar_initialization_info[15];
            Bg.z() = _lidar_initialization_info[16];
            // Gravity
            gravity = _lidar_initialization_info[17];
        };

        map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>> > > points;
        double t;
        
        IntegrationBase *pre_integration;
        bool is_key_frame;

        // Lidar odometry info
        int reset_id;
        Vector3d T;
        Matrix3d R;
        Vector3d V;
        Vector3d Ba;
        Vector3d Bg;
        double gravity;
};


bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);


class OdometryRegister
{
public:

    Eigen::Affine3d depth_frame_from_lidar = Eigen::Affine3d::Identity();

    explicit OdometryRegister(const std::shared_ptr<rclcpp::Node> &node)
        : logger_(node->get_logger()), clock_(node->get_clock())
    {
        const double tx = node->get_parameter("lidar_to_cam_tx").as_double();
        const double ty = node->get_parameter("lidar_to_cam_ty").as_double();
        const double tz = node->get_parameter("lidar_to_cam_tz").as_double();
        const double rx = node->get_parameter("lidar_to_cam_rx").as_double();
        const double ry = node->get_parameter("lidar_to_cam_ry").as_double();
        const double rz = node->get_parameter("lidar_to_cam_rz").as_double();
        depth_frame_from_lidar = lvi_sam::visual_frames::depth_frame_from_lidar(
            tx, ty, tz, rx, ry, rz);
    }

    // convert odometry from ROS Lidar frame to VINS camera frame
    vector<float> getOdometry(deque<nav_msgs::msg::Odometry>& odomQueue, double img_time)
    {
        vector<float> odometry_channel;
        odometry_channel.resize(18, -1); // reset id(1), P(3), Q(4), V(3), Ba(3), Bg(3), gravity(1)

        nav_msgs::msg::Odometry odomCur;
        
        // pop old odometry msg
        while (!odomQueue.empty()) 
        {
            if (rclcpp::Time(odomQueue.front().header.stamp).seconds() < img_time - 0.05)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
        {
            return odometry_channel;
        }

        // find the odometry time that is the closest to image time
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            odomCur = odomQueue[i];

            if (rclcpp::Time(odomCur.header.stamp).seconds() < img_time - 0.002) // 500Hz imu
                continue;
            else
                break;
        }

        // time stamp difference still too large
        if (abs(rclcpp::Time(odomCur.header.stamp).seconds() - img_time) > 0.05)
        {
            return odometry_channel;
        }

        const auto rejectInvalidPrior = [this, &odometry_channel](const char* reason) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000,
                "Rejecting LiDAR initialization prior: %s", reason);
            return odometry_channel;
        };
        for (std::size_t index = 0;
             index <= lvi_sam::internal_odom_metadata::visual_prior::kLastRequiredIndex;
             ++index)
        {
            if (!std::isfinite(odomCur.pose.covariance[index]))
                return rejectInvalidPrior("non-finite internal metadata");
        }
        const double resetId = odomCur.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kResetId];
        const double gravity = odomCur.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kGravity];
        if (resetId < 0.0 || std::abs(resetId - std::round(resetId)) > 1e-6)
            return rejectInvalidPrior("invalid reset identifier");
        if (gravity < 5.0 || gravity > 15.0)
            return rejectInvalidPrior("missing or implausible gravity metadata");

        const auto& position = odomCur.pose.pose.position;
        const auto& orientation = odomCur.pose.pose.orientation;
        const auto& velocity = odomCur.twist.twist.linear;
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
            !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
            !std::isfinite(orientation.w) || !std::isfinite(velocity.x) ||
            !std::isfinite(velocity.y) || !std::isfinite(velocity.z))
            return rejectInvalidPrior("non-finite pose or velocity");

        // Convert the ROS odometry pose of the physical LiDAR into the VINS
        // world pose of the optical-style camera/body.  The calibrated
        // LiDAR->depth transform and the fixed depth->VINS axis convention are
        // deliberately separate; visualization and depth injection use the
        // same definitions from visual_frame_conventions.hpp.
        Eigen::Quaterniond q_odom_lidar(
            odomCur.pose.pose.orientation.w,
            odomCur.pose.pose.orientation.x,
            odomCur.pose.pose.orientation.y,
            odomCur.pose.pose.orientation.z);
        if (q_odom_lidar.norm() < 1e-9) return odometry_channel;
        q_odom_lidar.normalize();
        Eigen::Affine3d odom_from_lidar = Eigen::Affine3d::Identity();
        odom_from_lidar.linear() = q_odom_lidar.toRotationMatrix();
        odom_from_lidar.translation() = Eigen::Vector3d(
            odomCur.pose.pose.position.x,
            odomCur.pose.pose.position.y,
            odomCur.pose.pose.position.z);
        const Eigen::Affine3d vins_camera_from_lidar =
            lvi_sam::visual_frames::vins_camera_from_depth_frame() *
            depth_frame_from_lidar;
        const Eigen::Affine3d vins_world_from_camera =
            lvi_sam::visual_frames::vins_world_from_ros_odom() *
            odom_from_lidar * vins_camera_from_lidar.inverse();
        Eigen::Quaterniond q_vins_world_camera(vins_world_from_camera.rotation());
        q_vins_world_camera.normalize();
        odomCur.pose.pose.orientation.x = q_vins_world_camera.x();
        odomCur.pose.pose.orientation.y = q_vins_world_camera.y();
        odomCur.pose.pose.orientation.z = q_vins_world_camera.z();
        odomCur.pose.pose.orientation.w = q_vins_world_camera.w();
        odomCur.pose.pose.position.x = vins_world_from_camera.translation().x();
        odomCur.pose.pose.position.y = vins_world_from_camera.translation().y();
        odomCur.pose.pose.position.z = vins_world_from_camera.translation().z();

        // imuPreintegration publishes currentState.velocity() in the odometry
        // (world) frame.  Changing the child pose from lidar to camera must not
        // rotate that world-frame velocity by the sensor extrinsic.  A lever-arm
        // velocity correction would additionally require a consistently framed
        // angular velocity; for the present tightly mounted sensors, retain the
        // IMU/world velocity as the initialization prior, transformed only by
        // the fixed ROS-odom -> VINS-world convention.
        const Eigen::Vector3d velocity_ros_odom(
            odomCur.twist.twist.linear.x,
            odomCur.twist.twist.linear.y,
            odomCur.twist.twist.linear.z);
        const Eigen::Vector3d velocity_vins_world =
            lvi_sam::visual_frames::vins_world_from_ros_odom().linear() *
            velocity_ros_odom;
        odomCur.twist.twist.linear.x = velocity_vins_world.x();
        odomCur.twist.twist.linear.y = velocity_vins_world.y();
        odomCur.twist.twist.linear.z = velocity_vins_world.z();

        odometry_channel[0] = odomCur.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kResetId];
        odometry_channel[1] = odomCur.pose.pose.position.x;
        odometry_channel[2] = odomCur.pose.pose.position.y;
        odometry_channel[3] = odomCur.pose.pose.position.z;
        odometry_channel[4] = odomCur.pose.pose.orientation.x;
        odometry_channel[5] = odomCur.pose.pose.orientation.y;
        odometry_channel[6] = odomCur.pose.pose.orientation.z;
        odometry_channel[7] = odomCur.pose.pose.orientation.w;
        odometry_channel[8]  = odomCur.twist.twist.linear.x;
        odometry_channel[9]  = odomCur.twist.twist.linear.y;
        odometry_channel[10] = odomCur.twist.twist.linear.z;
        odometry_channel[11] = odomCur.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kAccelBiasX];
        odometry_channel[12] = odomCur.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kAccelBiasY];
        odometry_channel[13] = odomCur.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kAccelBiasZ];
        odometry_channel[14] = odomCur.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kGyroBiasX];
        odometry_channel[15] = odomCur.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kGyroBiasY];
        odometry_channel[16] = odomCur.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kGyroBiasZ];
        odometry_channel[17] = odomCur.pose.covariance[
            lvi_sam::internal_odom_metadata::visual_prior::kGravity];

        return odometry_channel;
    }

private:
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
};
