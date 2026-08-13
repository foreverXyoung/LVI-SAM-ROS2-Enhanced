#include "parameters.h"
#include "lvi_sam/package_assets.hpp"

#include <limits>
#include <stdexcept>

std::string PROJECT_NAME;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;
double IMU_ACCELERATION_SCALE;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string IMU_TOPIC;
std::string ODOM_TOPIC;
int ROW, COL;
double TD, TR;

int USE_LIDAR;
int USE_LIDAR_ODOMETRY_PRIOR;
int ALIGN_CAMERA_LIDAR_COORDINATE;


void readParameters(std::shared_ptr<rclcpp::Node> n)
{
    std::string config_file;
    n->declare_parameter("vins_config_file", "");
    n->get_parameter("vins_config_file", config_file);
    // cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    // if(!fsSettings.isOpened())
    // {
    //     std::cerr << "ERROR: Wrong path to settings" << std::endl;
    // }

    n->declare_parameter("PROJECT_NAME", "");
    n->get_parameter("PROJECT_NAME", PROJECT_NAME);
    if (PROJECT_NAME.empty())
        throw std::runtime_error("PROJECT_NAME must not be empty");
    // fsSettings["project_name"] >> PROJECT_NAME;
    // PROJECT_NAME controls topic names only; package assets always belong to lvi_sam.
    config_file = lvi_sam::resolve_package_asset(
        config_file, "camera configuration");

    n->declare_parameter("imu_topic", "");
    n->get_parameter("imu_topic", IMU_TOPIC);
    if (IMU_TOPIC.empty())
        throw std::runtime_error("imu_topic must not be empty");
    n->declare_parameter("imuAccelerationScale", 1.0);
    n->get_parameter("imuAccelerationScale", IMU_ACCELERATION_SCALE);
    if (!std::isfinite(IMU_ACCELERATION_SCALE) ||
        IMU_ACCELERATION_SCALE <= 0.0)
        throw std::runtime_error(
            "imuAccelerationScale must be finite and greater than 0");
    n->declare_parameter("odom_topic", "/odometry/imu");
    n->get_parameter("odom_topic", ODOM_TOPIC);
    if (ODOM_TOPIC.empty())
        throw std::runtime_error("odom_topic must not be empty");
    // fsSettings["imu_topic"] >> IMU_TOPIC;

    n->declare_parameter("use_lidar", 1);
    n->get_parameter("use_lidar", USE_LIDAR);
    n->declare_parameter("use_lidar_odometry_prior", 0);
    n->get_parameter(
        "use_lidar_odometry_prior", USE_LIDAR_ODOMETRY_PRIOR);
    n->declare_parameter("align_camera_lidar_estimation", 1);
    n->get_parameter("align_camera_lidar_estimation", ALIGN_CAMERA_LIDAR_COORDINATE);
    if ((USE_LIDAR != 0 && USE_LIDAR != 1) ||
        (USE_LIDAR_ODOMETRY_PRIOR != 0 &&
         USE_LIDAR_ODOMETRY_PRIOR != 1) ||
        (ALIGN_CAMERA_LIDAR_COORDINATE != 0 && ALIGN_CAMERA_LIDAR_COORDINATE != 1))
        throw std::runtime_error(
            "use_lidar, use_lidar_odometry_prior and "
            "align_camera_lidar_estimation must be 0 or 1");
    if (ALIGN_CAMERA_LIDAR_COORDINATE == 1 && USE_LIDAR == 0)
        throw std::runtime_error(
            "align_camera_lidar_estimation requires use_lidar=1");
    const double missing_extrinsic =
        std::numeric_limits<double>::quiet_NaN();
    for (const char* parameter_name : {
             "lidar_to_cam_tx", "lidar_to_cam_ty", "lidar_to_cam_tz",
             "lidar_to_cam_rx", "lidar_to_cam_ry", "lidar_to_cam_rz"}) {
        n->declare_parameter(parameter_name, missing_extrinsic);
        if (!std::isfinite(n->get_parameter(parameter_name).as_double())) {
            throw std::runtime_error(
                std::string(parameter_name) +
                " must be explicitly configured with a finite value; no "
                "identity LiDAR-camera extrinsic is assumed");
        }
    }
    // fsSettings["use_lidar"] >> USE_LIDAR;
    // fsSettings["align_camera_lidar_estimation"] >> ALIGN_CAMERA_LIDAR_COORDINATE;

    n->declare_parameter("max_solver_time", 1.0);
    n->get_parameter("max_solver_time", SOLVER_TIME);
    n->declare_parameter("max_num_iterations", 1);
    n->get_parameter("max_num_iterations", NUM_ITERATIONS);
    n->declare_parameter("keyframe_parallax", 1.0);
    n->get_parameter("keyframe_parallax", MIN_PARALLAX);
    if (!std::isfinite(SOLVER_TIME) || SOLVER_TIME <= 0.0)
        throw std::runtime_error("max_solver_time must be finite and greater than 0");
    if (NUM_ITERATIONS <= 0)
        throw std::runtime_error("max_num_iterations must be greater than 0");
    if (!std::isfinite(MIN_PARALLAX) || MIN_PARALLAX <= 0.0)
        throw std::runtime_error("keyframe_parallax must be finite and greater than 0");
    // SOLVER_TIME = fsSettings["max_solver_time"];
    // NUM_ITERATIONS = fsSettings["max_num_iterations"];
    // MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    n->declare_parameter("acc_n", 1.0);
    n->get_parameter("acc_n", ACC_N);
    n->declare_parameter("acc_w", 1.0);
    n->get_parameter("acc_w", ACC_W);
    n->declare_parameter("gyr_n", 1.0);
    n->get_parameter("gyr_n", GYR_N);
    n->declare_parameter("gyr_w", 1.0);
    n->get_parameter("gyr_w", GYR_W);
    n->declare_parameter("g_norm", 1.0);
    n->get_parameter("g_norm", G.z());
    n->declare_parameter("image_height", 720);
    n->get_parameter("image_height", ROW);
    n->declare_parameter("image_width", 540);
    n->get_parameter("image_width", COL);
    if (!std::isfinite(ACC_N) || ACC_N <= 0.0 ||
        !std::isfinite(ACC_W) || ACC_W <= 0.0 ||
        !std::isfinite(GYR_N) || GYR_N <= 0.0 ||
        !std::isfinite(GYR_W) || GYR_W <= 0.0)
        throw std::runtime_error(
            "acc_n, acc_w, gyr_n and gyr_w must be finite and greater than 0");
    if (!std::isfinite(G.z()) || G.z() <= 0.0)
        throw std::runtime_error("g_norm must be finite and greater than 0");
    if (ROW <= 0 || COL <= 0)
        throw std::runtime_error("image_height and image_width must be greater than 0");
    // ACC_N = fsSettings["acc_n"];
    // ACC_W = fsSettings["acc_w"];
    // GYR_N = fsSettings["gyr_n"];
    // GYR_W = fsSettings["gyr_w"];
    // G.z() = fsSettings["g_norm"];
    // ROW = fsSettings["image_height"];
    // COL = fsSettings["image_width"];
    RCLCPP_INFO(n->get_logger(), "Image dimention: ROW: %d COL: %d ", ROW, COL);

    n->declare_parameter("estimate_extrinsic", 1);
    n->get_parameter("estimate_extrinsic", ESTIMATE_EXTRINSIC);
    if (ESTIMATE_EXTRINSIC < 0 || ESTIMATE_EXTRINSIC > 2)
        throw std::runtime_error("estimate_extrinsic must be 0, 1 or 2");
    RIC.clear();
    TIC.clear();
    // ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        RCLCPP_INFO(n->get_logger(), "have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            RCLCPP_INFO(n->get_logger(), " Optimize extrinsic param around initial guess!");
        }
        if (ESTIMATE_EXTRINSIC == 0)
            RCLCPP_INFO(n->get_logger(), " Fix extrinsic param.");

        cv::Mat cv_R, cv_T;
        std::vector<double> cv_T_vec, cv_R_vec;
        n->declare_parameter("extrinsicRotation", std::vector<double>(9, 0.0));
        n->get_parameter("extrinsicRotation", cv_R_vec);
        n->declare_parameter("extrinsicTranslation", std::vector<double>(3, 0.0));
        n->get_parameter("extrinsicTranslation", cv_T_vec);
        if (cv_R_vec.size() != 9 || cv_T_vec.size() != 3)
            throw std::runtime_error(
                "extrinsicRotation/extrinsicTranslation must contain 9/3 values");
        cv_R = cv::Mat(3, 3, CV_64F, cv_R_vec.data()).clone(); 
        cv_T = cv::Mat(3, 1, CV_64F, cv_T_vec.data()).clone(); 
        // fsSettings["extrinsicRotation"] >> cv_R;
        // fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        if (!eigen_R.allFinite() || !eigen_T.allFinite() ||
            !(eigen_R.transpose() * eigen_R)
                 .isApprox(Eigen::Matrix3d::Identity(), 1e-3) ||
            std::abs(eigen_R.determinant() - 1.0) > 1e-3)
            throw std::runtime_error(
                "camera/IMU extrinsic must be a finite rigid transform");
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized().toRotationMatrix();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        RCLCPP_INFO_STREAM(n->get_logger(), "Extrinsic_R : " << std::endl << RIC[0]);
        RCLCPP_INFO_STREAM(n->get_logger(), "Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    
    n->declare_parameter("td", 0.0);
    n->get_parameter("td", TD);
    n->declare_parameter("estimate_td", 1);
    n->get_parameter("estimate_td", ESTIMATE_TD);
    if (ESTIMATE_TD != 0 && ESTIMATE_TD != 1)
        throw std::runtime_error("estimate_td must be 0 or 1");
    if (!std::isfinite(TD))
        throw std::runtime_error("td must be finite");
    // TD = fsSettings["td"];
    // ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        RCLCPP_INFO_STREAM(n->get_logger(), "Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        RCLCPP_INFO_STREAM(n->get_logger(), "Synchronized sensors, fix time offset: " << TD);

    n->declare_parameter("rolling_shutter", 1);
    n->get_parameter("rolling_shutter", ROLLING_SHUTTER);
    if (ROLLING_SHUTTER != 0 && ROLLING_SHUTTER != 1)
        throw std::runtime_error("rolling_shutter must be 0 or 1");
    // ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        // TR = fsSettings["rolling_shutter_tr"];
        n->declare_parameter("rolling_shutter_tr", 0.0);
        n->get_parameter("rolling_shutter_tr", TR);
        if (!std::isfinite(TR) || TR < 0.0)
            throw std::runtime_error(
                "rolling_shutter_tr must be finite and greater than or equal to 0");
        RCLCPP_INFO_STREAM(n->get_logger(), "rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    // fsSettings.release();
}
