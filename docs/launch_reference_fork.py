import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('liosam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description_file = LaunchConfiguration('robot_description_file')
    start_robot_state_publisher = LaunchConfiguration(
        'start_robot_state_publisher')
    navsat_x = LaunchConfiguration('navsat_x')
    navsat_y = LaunchConfiguration('navsat_y')
    navsat_z = LaunchConfiguration('navsat_z')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')
    start_static_map_to_odom = LaunchConfiguration('start_static_map_to_odom')
    save_pcd = LaunchConfiguration('save_pcd')
    pcd_directory = LaunchConfiguration('pcd_directory')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'liosam_params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use the /clock topic for rosbag or simulation playback.')

    robot_description_declare = DeclareLaunchArgument(
        'robot_description_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('fairino_description'),
            'urdf',
            'mobile_base.urdf.xacro',
        ]),
        description='Path to the robot URDF/xacro file to publish.')

    start_robot_state_publisher_declare = DeclareLaunchArgument(
        'start_robot_state_publisher',
        default_value='true',
        description='Start the TF publisher for the selected robot model.')

    navsat_x_declare = DeclareLaunchArgument(
        'navsat_x',
        default_value='0.0',
        description='Measured base_link to primary RTK antenna X offset.')

    navsat_y_declare = DeclareLaunchArgument(
        'navsat_y',
        default_value='0.0',
        description='Measured base_link to primary RTK antenna Y offset.')

    navsat_z_declare = DeclareLaunchArgument(
        'navsat_z',
        default_value='0.0',
        description='Measured base_link to primary RTK antenna Z offset.')

    camera_x_declare = DeclareLaunchArgument(
        'camera_x',
        default_value='0.5',
        description='Measured base_link to front camera X offset.')
    camera_y_declare = DeclareLaunchArgument(
        'camera_y',
        default_value='0.0',
        description='Measured base_link to front camera Y offset.')
    camera_z_declare = DeclareLaunchArgument(
        'camera_z',
        default_value='0.15',
        description='Measured base_link to front camera Z offset.')
    camera_roll_declare = DeclareLaunchArgument(
        'camera_roll',
        default_value='0.0',
        description='Measured front camera mount roll.')
    camera_pitch_declare = DeclareLaunchArgument(
        'camera_pitch',
        default_value='0.0',
        description='Measured front camera mount pitch.')
    camera_yaw_declare = DeclareLaunchArgument(
        'camera_yaw',
        default_value='0.0',
        description='Measured front camera mount yaw.')

    static_tf_declare = DeclareLaunchArgument(
        'start_static_map_to_odom',
        default_value='true',
        description='Publish the fallback zero map to odom transform.')

    save_pcd_declare = DeclareLaunchArgument(
        'save_pcd',
        default_value='false',
        description='Override savePCD for mapOptimization.')

    pcd_directory_declare = DeclareLaunchArgument(
        'pcd_directory',
        default_value='/home/lighter/Documents/return_station_ws/src/lio_sam_enhanced/map/',
        description=(
            'PCD output directory in mapping mode and input directory in '
            'localization mode.'))

    return LaunchDescription([
        params_declare,
        use_sim_time_declare,
        robot_description_declare,
        start_robot_state_publisher_declare,
        navsat_x_declare,
        navsat_y_declare,
        navsat_z_declare,
        camera_x_declare,
        camera_y_declare,
        camera_z_declare,
        camera_roll_declare,
        camera_pitch_declare,
        camera_yaw_declare,
        static_tf_declare,
        save_pcd_declare,
        pcd_directory_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file, {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            condition=IfCondition(start_static_map_to_odom),
            output='screen'
            ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            condition=IfCondition(start_robot_state_publisher),
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'robot_description': ParameterValue(
                    Command([
                        'xacro', ' ', robot_description_file,
                        ' navsat_x:=', navsat_x,
                        ' navsat_y:=', navsat_y,
                        ' navsat_z:=', navsat_z,
                        ' camera_x:=', camera_x,
                        ' camera_y:=', camera_y,
                        ' camera_z:=', camera_z,
                        ' camera_roll:=', camera_roll,
                        ' camera_pitch:=', camera_pitch,
                        ' camera_yaw:=', camera_yaw,
                    ]),
                    value_type=str,
                )
            }]
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file, {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file, {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'savePCD': ParameterValue(save_pcd, value_type=bool),
                'savePCDDirectory': pcd_directory,
                'Loc.loadPCDDirectory': pcd_directory,
            }],
            output='screen'
        ),
#        Node(
#            package='rviz2',
#            executable='rviz2',
#            name='rviz2',
#            arguments=['-d', rviz_config_file],
#            output='screen'
#        )
    ])
