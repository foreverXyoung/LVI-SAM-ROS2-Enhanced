#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <rclcpp/rclcpp.hpp>
#include <map>
#include "../feature_manager.h"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include "geometry_msgs/msg/quaternion.hpp"
#include <pcl/common/transforms.h>

using namespace Eigen;
using namespace std;

class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>>& _points, 
                   const vector<float> &_lidar_initialization_info,
                   double _t):
        t{_t}, is_key_frame{false}, reset_id{-1}, gravity{9.805}
        {
            points = _points;
            
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


class odometryRegister
{
public:

    std::shared_ptr<rclcpp::Node> node;
    Eigen::Affine3d camera_from_lidar = Eigen::Affine3d::Identity();
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_latest_odometry;

    odometryRegister(std::shared_ptr<rclcpp::Node> node)
    {
        const float tx = static_cast<float>(node->get_parameter("lidar_to_cam_tx").as_double());
        const float ty = static_cast<float>(node->get_parameter("lidar_to_cam_ty").as_double());
        const float tz = static_cast<float>(node->get_parameter("lidar_to_cam_tz").as_double());
        const float rx = static_cast<float>(node->get_parameter("lidar_to_cam_rx").as_double());
        const float ry = static_cast<float>(node->get_parameter("lidar_to_cam_ry").as_double());
        const float rz = static_cast<float>(node->get_parameter("lidar_to_cam_rz").as_double());
        camera_from_lidar = pcl::getTransformation(tx, ty, tz, rx, ry, rz).cast<double>();
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

        // Apply the calibrated full SE(3) transform. camera_from_lidar maps
        // lidar-frame points into the camera/body frame, so a camera pose is
        // T_odom_lidar * inverse(T_camera_lidar).
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
        const Eigen::Affine3d odom_from_camera =
            odom_from_lidar * camera_from_lidar.inverse();
        const Eigen::Quaterniond q_odom_cam(odom_from_camera.rotation());
        odomCur.pose.pose.orientation.x = q_odom_cam.x();
        odomCur.pose.pose.orientation.y = q_odom_cam.y();
        odomCur.pose.pose.orientation.z = q_odom_cam.z();
        odomCur.pose.pose.orientation.w = q_odom_cam.w();
        odomCur.pose.pose.position.x = odom_from_camera.translation().x();
        odomCur.pose.pose.position.y = odom_from_camera.translation().y();
        odomCur.pose.pose.position.z = odom_from_camera.translation().z();

        // imuPreintegration publishes currentState.velocity() in the odometry
        // (world) frame.  Changing the child pose from lidar to camera must not
        // rotate that world-frame velocity by the sensor extrinsic.  A lever-arm
        // velocity correction would additionally require a consistently framed
        // angular velocity; for the present tightly mounted sensors, retain the
        // IMU/world velocity as the initialization prior.

        // odomCur.header.stamp = ros::Time().fromSec(img_time);
        // odomCur.header.frame_id = "vins_world";
        // odomCur.child_frame_id = "vins_body";
        // pub_latest_odometry.publish(odomCur);

        odometry_channel[0] = odomCur.pose.covariance[0];
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
        odometry_channel[11] = odomCur.pose.covariance[1];
        odometry_channel[12] = odomCur.pose.covariance[2];
        odometry_channel[13] = odomCur.pose.covariance[3];
        odometry_channel[14] = odomCur.pose.covariance[4];
        odometry_channel[15] = odomCur.pose.covariance[5];
        odometry_channel[16] = odomCur.pose.covariance[6];
        odometry_channel[17] = odomCur.pose.covariance[7];

        return odometry_channel;
    }
};
