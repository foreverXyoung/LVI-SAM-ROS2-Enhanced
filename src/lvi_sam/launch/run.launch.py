# LVI-SAM-ROS2-Enhanced 入口 launch
# 启动：LIS(激光, 2 节点) + VIS(视觉, 3 节点)，并通过话题接线完成"话题级松耦合"。
#
# 接线要点（已源码核对）：
#   ① LIS→VIS 位姿/尺度先验：两侧由 odom_topic 统一为绝对话题 /odometry/imu
#   ② LIS→VIS 激光深度：feature_tracker 订阅 POINT_CLOUD_TOPIC(= /lio_sam/deskew/cloud_deskewed，yaml 配)
#   ③b VIS→LIS 视觉回环候选：loop 发布 /lvi_sam/vins/loop/match_frame
#                              → mapOptimization 订阅 lio_loop/loop_closure_detection 经 remap 接收
#   （③a VIS→LIS 前端初值猜测 imu_propagate_ros 在 fork 前端未订阅，最小闭环暂不接）

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
import os
import re
import xacro
import yaml


def load_ros_parameters(path):
    """Load the wildcard ROS parameter mapping used by project YAML files."""
    try:
        with open(path, 'r', encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        parameters = document['/**']['ros__parameters']
    except (OSError, UnicodeError, yaml.YAMLError, KeyError, TypeError) as exc:
        raise RuntimeError(
            'Cannot read ROS parameter file ' + path + ': ' + str(exc)) from exc
    if not isinstance(parameters, dict):
        raise RuntimeError('ROS parameter mapping is invalid: ' + path)
    return parameters


def generate_launch_description():
    pkg_dir = get_package_share_directory('lvi_sam')
    pkg_prefix = get_package_prefix('lvi_sam')

    # ---- 可配置参数 ----
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='mapping',
        description='运行模式：mapping 或 localization')

    scene_arg = DeclareLaunchArgument(
        'scene', default_value='generic',
        description='配置场景：generic、charging 或 gazebo')

    lidar_params_arg = DeclareLaunchArgument(
        'lidar_params_file',
        default_value='',
        description='可选 LIS 参数文件；非空时覆盖 mode/scene 自动选择')

    camera_params_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value='',
        description=(
            'Optional VIS(camera) profile. Empty selects params_camera.yaml '
            'for external IMU or params_camera_mid360.yaml for MID-360 IMU'))

    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/color/image_raw',
        description='VIS 输入图像话题；覆盖 camera 参数文件中的 image_topic')

    imu_source_arg = DeclareLaunchArgument(
        'imu_source', default_value='external',
        description='IMU profile: external (/IMU_data) or mid360 (/livox/imu)')

    imu_params_arg = DeclareLaunchArgument(
        'imu_params_file', default_value='',
        description=(
            'Optional absolute custom IMU profile; when non-empty it '
            'overrides imu_source profile selection'))

    mount_params_arg = DeclareLaunchArgument(
        'mount_params_file',
        default_value=os.path.join(
            pkg_dir, 'config', 'params_mount_robot.yaml'),
        description=(
            'Robot base-to-LiDAR mounting calibration; independent from the '
            'selected IMU profile'))

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='',
        description='Optional advanced topic override; normally use imu_source')

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odometry/imu',
        description='LIS IMU preintegration output and VIS odometry-prior input')

    project_name_arg = DeclareLaunchArgument(
        'project_name', default_value='lvi_sam',
        description='Visual pipeline topic root; publishes under /<name>/vins')

    gps_topic_arg = DeclareLaunchArgument(
        'gps_topic', default_value='',
        description='可选 map 对齐 RTK/GPS nav_msgs/Odometry 话题；非空时覆盖 YAML')

    enable_visual_arg = DeclareLaunchArgument(
        'enable_visual', default_value='true',
        description='是否启动 visual_feature、estimator 和 loop 三个视觉节点')

    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz', default_value='true',
        description='是否使用工程内置配置启动 RViz2')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_dir, 'config', 'rviz2.rviz'),
        description='RViz2 配置文件')

    rviz_fixed_frame_arg = DeclareLaunchArgument(
        'rviz_fixed_frame', default_value='odom',
        description='RViz2 Fixed Frame；当前算法输出默认使用 odom')

    robot_description_arg = DeclareLaunchArgument(
        'robot_description_file', default_value='',
        description='可选 URDF/Xacro；非空时由本 launch 启动 robot_state_publisher')

    pcd_dir_arg = DeclareLaunchArgument(
        'pcd_directory',
        default_value='/tmp/lvi_sam_maps',
        description='先验地图读取 / 建图输出目录（覆盖 yaml 内的硬编码默认）')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='是否使用仿真时间（Gazebo 下置 true）')

    publish_map_odom_arg = DeclareLaunchArgument(
        'publish_map_odom_static', default_value='false',
        description='是否发布 map->odom 静态变换；仅在没有其他 map->odom 发布者时启用')

    publish_fused_tf_arg = DeclareLaunchArgument(
        'publish_fused_tf', default_value='true',
        description='是否发布 odom->base_link；关闭时算法不查询平台 TF tree')

    def launch_setup(context):
        mode = LaunchConfiguration('mode').perform(context).strip().lower()
        scene = LaunchConfiguration('scene').perform(context).strip().lower()
        lidar_params = LaunchConfiguration('lidar_params_file').perform(context)
        camera_params = LaunchConfiguration(
            'camera_params_file').perform(context).strip()
        camera_params_explicit = bool(camera_params)
        image_topic = LaunchConfiguration('image_topic').perform(context).strip()
        imu_source = LaunchConfiguration('imu_source').perform(
            context).strip().lower()
        imu_params = LaunchConfiguration(
            'imu_params_file').perform(context).strip()
        mount_params = LaunchConfiguration(
            'mount_params_file').perform(context).strip()
        imu_topic = LaunchConfiguration('imu_topic').perform(context).strip()
        odom_topic = LaunchConfiguration('odom_topic').perform(context).strip()
        project_name = LaunchConfiguration('project_name').perform(
            context).strip().strip('/')
        gps_topic = LaunchConfiguration('gps_topic').perform(context).strip()
        enable_visual = LaunchConfiguration('enable_visual').perform(
            context).strip().lower()
        enable_rviz = LaunchConfiguration('enable_rviz').perform(
            context).strip().lower()
        rviz_config = LaunchConfiguration('rviz_config_file').perform(context)
        rviz_fixed_frame = LaunchConfiguration(
            'rviz_fixed_frame').perform(context)
        robot_description_file = LaunchConfiguration(
            'robot_description_file').perform(context)
        pcd_directory = LaunchConfiguration('pcd_directory').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time').perform(
            context).strip().lower()
        publish_map_odom = LaunchConfiguration(
            'publish_map_odom_static').perform(context).strip().lower()
        publish_fused_tf = LaunchConfiguration(
            'publish_fused_tf').perform(context).strip().lower()

        if mode not in ('mapping', 'localization'):
            raise RuntimeError('mode must be mapping or localization: ' + mode)
        if scene not in ('generic', 'charging', 'gazebo'):
            raise RuntimeError('scene must be generic, charging or gazebo: ' + scene)
        if not imu_params and imu_source not in ('external', 'mid360'):
            raise RuntimeError(
                'imu_source must be external or mid360: ' + imu_source)
        if enable_visual not in ('true', 'false'):
            raise RuntimeError('enable_visual must be true or false')
        if enable_rviz not in ('true', 'false'):
            raise RuntimeError('enable_rviz must be true or false')
        if publish_fused_tf not in ('true', 'false'):
            raise RuntimeError('publish_fused_tf must be true or false')
        if publish_map_odom not in ('true', 'false'):
            raise RuntimeError(
                'publish_map_odom_static must be true or false')
        if use_sim_time not in ('true', 'false'):
            raise RuntimeError('use_sim_time must be true or false')
        if not odom_topic.strip():
            raise RuntimeError('odom_topic must not be empty')
        if not project_name:
            raise RuntimeError('project_name must not be empty')
        if not re.fullmatch(
                r'[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*',
                project_name):
            raise RuntimeError(
                "project_name must contain valid ROS namespace segments")
        if enable_visual == 'true' and not image_topic.strip():
            raise RuntimeError(
                'image_topic must not be empty when visual nodes are enabled')
        if not pcd_directory.strip() or not os.path.isabs(pcd_directory):
            raise RuntimeError('pcd_directory must be an absolute path')
        if enable_rviz == 'true' and not rviz_fixed_frame.strip():
            raise RuntimeError(
                'rviz_fixed_frame must not be empty when RViz is enabled')
        if not lidar_params:
            config_name = (
                'params_' + mode + '.yaml'
                if scene == 'generic'
                else 'params_' + scene + '_' + mode + '.yaml')
            lidar_params = os.path.join(pkg_dir, 'config', config_name)
        if not os.path.isabs(lidar_params):
            raise RuntimeError('lidar_params_file must be an absolute path')
        if not os.path.isfile(lidar_params):
            raise RuntimeError('LIS parameter file does not exist: ' + lidar_params)

        imu_profile = (
            imu_params if imu_params else os.path.join(
                pkg_dir, 'config', 'params_imu_' + imu_source + '.yaml'))
        if not os.path.isabs(imu_profile):
            raise RuntimeError('imu_params_file must be an absolute path')
        if not os.path.isfile(imu_profile):
            raise RuntimeError('IMU profile does not exist: ' + imu_profile)
        if not camera_params_explicit:
            if imu_params and enable_visual == 'true':
                raise RuntimeError(
                    'A custom imu_params_file with enable_visual=true also '
                    'requires an explicit camera_params_file for the same IMU')
            camera_profile_name = (
                'params_camera_mid360.yaml'
                if not imu_params and imu_source == 'mid360'
                else 'params_camera.yaml')
            camera_params = os.path.join(
                pkg_dir, 'config', camera_profile_name)
        if not os.path.isabs(mount_params):
            raise RuntimeError('mount_params_file must be an absolute path')
        if not os.path.isfile(mount_params):
            raise RuntimeError(
                'Mounting parameter file does not exist: ' + mount_params)
        imu_profile_parameters = load_ros_parameters(imu_profile)
        profile_imu_topic = str(
            imu_profile_parameters.get('imuTopic', '')).strip()
        selected_imu_topic = imu_topic if imu_topic else profile_imu_topic
        if not selected_imu_topic:
            raise RuntimeError(
                'Selected IMU profile does not define a non-empty imuTopic')
        for parameter_name, topic in (
                ('imu_topic', selected_imu_topic),
                ('odom_topic', odom_topic), ('image_topic', image_topic),
                ('gps_topic', gps_topic)):
            if topic.strip() and not topic.strip().startswith('/'):
                raise RuntimeError(parameter_name + ' must be an absolute topic')

        if enable_visual == 'true':
            if not os.path.isabs(camera_params):
                raise RuntimeError('camera_params_file must be an absolute path')
            if not os.path.isfile(camera_params):
                raise RuntimeError('VIS parameter file does not exist: ' + camera_params)
            nominal_mid360_camera = os.path.realpath(os.path.join(
                pkg_dir, 'config', 'params_camera_mid360.yaml'))
            if os.path.realpath(camera_params) == nominal_mid360_camera:
                get_logger('lvi_sam.launch').warning(
                    'Using nominal MID-360 camera extrinsics with LiDAR depth '
                    'and odometry prior disabled. Calibrate camera-IMU/LiDAR '
                    'extrinsics before accuracy evaluation.')
            missing_visual_nodes = [
                executable for executable in (
                    'visual_feature_node',
                    'visual_estimator_node',
                    'visual_loop_node',
                ) if not os.path.isfile(os.path.join(
                    pkg_prefix, 'lib', 'lvi_sam', executable))
            ]
            if missing_visual_nodes:
                raise RuntimeError(
                    'enable_visual=true, but this installation was built '
                    'without VIS executables: ' + ', '.join(missing_visual_nodes) +
                    '. Rebuild with -DBUILD_VISUAL=ON or launch with '
                    'enable_visual:=false.')
        if enable_rviz == 'true':
            if not os.path.isabs(rviz_config):
                raise RuntimeError('rviz_config_file must be an absolute path')
            if not os.path.isfile(rviz_config):
                raise RuntimeError('RViz configuration file does not exist: ' + rviz_config)

        # LIS 节点共享的覆盖参数（pcd 目录 + 仿真时间）
        # yaml 中 loadPCDDirectory 位于 Loc 参数组；launch_ros 会将该
        # 嵌套字典规范化为 Loc.loadPCDDirectory。
        lis_common = [
            {'use_sim_time': use_sim_time == 'true'},
            {'imuTopic': selected_imu_topic},
            {'odomTopic': odom_topic},
            {'publishFusedBaseTF': publish_fused_tf == 'true'},
            {'savePCDDirectory': pcd_directory},
            {'Loc': {'loadPCDDirectory': pcd_directory}},
        ]
        if gps_topic:
            lis_common.append({'gpsTopic': gps_topic})
        vis_common = [
            {'use_sim_time': use_sim_time == 'true'},
            {'imu_topic': selected_imu_topic},
            {'odom_topic': odom_topic},
            {'image_topic': image_topic},
            {'PROJECT_NAME': project_name},
            {'vins_config_file': camera_params},
        ]

        nodes = []

        if robot_description_file:
            if not os.path.isabs(robot_description_file):
                raise RuntimeError(
                    'robot_description_file must be an absolute path')
            if not os.path.isfile(robot_description_file):
                raise RuntimeError(
                    'robot_description_file does not exist: ' +
                    robot_description_file)
            robot_description = xacro.process_file(
                robot_description_file).toxml()
            nodes.append(Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time == 'true',
                }]))

        # map -> odom 静态变换（可选）
        if publish_map_odom == 'true':
            nodes.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                output='screen'))

        # ===================== LIS（激光，来自 fork）=====================
        nodes.append(Node(
            package='lvi_sam',
            executable='lvi_sam_imuPreintegration',
            name='lvi_sam_imuPreintegration',
            output='screen',
            parameters=[lidar_params, imu_profile, mount_params] + lis_common))

        nodes.append(Node(
            package='lvi_sam',
            executable='lvi_sam_mapOptimization',
            name='lvi_sam_mapOptimization',
            output='screen',
            parameters=[lidar_params, imu_profile, mount_params] + lis_common,
            # ③b：接收 VIS 视觉回环候选（VIS 发布 /lvi_sam/vins/loop/match_frame）
            remappings=[('lio_loop/loop_closure_detection',
                         '/' + project_name + '/vins/loop/match_frame')]))

        # ===================== VIS（视觉，来自 LVI-SAM-ROS2）=====================
        # 视觉三节点均读取 camera_params（含 PROJECT_NAME / imu_topic / image_topic /
        # point_cloud_topic / vocabulary_file 等）；资源路径由 lvi_sam 包共享目录解析。
        nodes.append(Node(
            package='lvi_sam',
            executable='visual_feature_node',
            name='visual_feature_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_visual')),
            parameters=[camera_params, imu_profile] + vis_common))

        nodes.append(Node(
            package='lvi_sam',
            executable='visual_estimator_node',
            name='visual_estimator_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_visual')),
            # ① LIS→VIS 里程计先验：发布端和订阅端共用 odom_topic。
            parameters=[camera_params, imu_profile] + vis_common))

        nodes.append(Node(
            package='lvi_sam',
            executable='visual_loop_node',
            name='visual_loop_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_visual')),
            parameters=[camera_params, imu_profile] + vis_common))

        # RViz is independent from the camera/VIS pipeline.  Keep it enabled
        # by default for on-robot validation, while allowing headless launches
        # to opt out with enable_rviz:=false.
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='lvi_sam_rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
            arguments=['-d', rviz_config, '-f', rviz_fixed_frame],
            parameters=[{'use_sim_time': use_sim_time == 'true'}]))

        return nodes

    return LaunchDescription([
        mode_arg,
        scene_arg,
        lidar_params_arg,
        camera_params_arg,
        image_topic_arg,
        imu_source_arg,
        imu_params_arg,
        mount_params_arg,
        imu_topic_arg,
        odom_topic_arg,
        project_name_arg,
        gps_topic_arg,
        enable_visual_arg,
        enable_rviz_arg,
        rviz_config_arg,
        rviz_fixed_frame_arg,
        robot_description_arg,
        pcd_dir_arg,
        use_sim_time_arg,
        publish_map_odom_arg,
        publish_fused_tf_arg,
        OpaqueFunction(function=launch_setup),
    ])
