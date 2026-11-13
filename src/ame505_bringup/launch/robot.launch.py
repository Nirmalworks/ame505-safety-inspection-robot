"""JetAuto hardware bringup using the URDF from ame505_description.

Starts: robot_state_publisher (our URDF), motor/IMU board, wheel odometry + EKF,
lidar (/scan), the Astra depth camera and Xbox teleop.

The Astra camera is owned by Hiwonder's ROS 1 container on the host, so the ROS 2
Orbbec driver can't open it (uvc_open "busy"). Instead the jetauto_toolkit
astra_bridge.py republishes it as /depth_cam/rgb/image_raw + /depth_cam/depth/image_raw
(depth registered to colour, 320x240 @ 6 Hz) in frame depth_cam_frame.

    ros2 launch ame505_bringup robot.launch.py use_rviz:=true   # drive + see lidar/camera
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ReplaceString

from ame505_bringup.paths import hiwonder_pkg


def include(pkg_path, rel, args=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, rel)),
        launch_arguments=(args or {}).items(), condition=condition)


def lidar(peripherals):
    """RPLidar A1 driver + Hiwonder's laser filter (front ~180 deg) -> /scan.

    Same settings as Hiwonder's peripherals/launch/include/sllidar_a1.launch.py, but the driver
    RESPAWNS: after a USB hiccup it died with SL_RESULT_OPERATION_TIMEOUT and stayed dead, so
    AMCL/Nav2 silently lost the robot. Other lidar types: Hiwonder's launch as before.
    """
    if os.environ.get('LIDAR_TYPE', 'A1') != 'A1':
        return [include(peripherals, 'launch/lidar.launch.py',
                        {'lidar_frame': 'lidar_frame', 'scan_topic': '/scan', 'scan_raw': '/scan_raw'})]
    return [
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node', output='screen',
             respawn=True, respawn_delay=3.0,
             parameters=[{'channel_type': 'serial', 'serial_baudrate': 115200,
                          'serial_port': '/dev/lidar', 'frame_id': 'lidar_frame',
                          'inverted': False, 'angle_compensate': True, 'scan_mode': 'Sensitivity'}],
             remappings=[('scan', '/scan_raw')]),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain', output='screen',
             parameters=[os.path.join(peripherals, 'config', 'lidar_filters_config_a1.yaml')],
             remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')]),
    ]


def generate_launch_description():
    desc = get_package_share_directory('ame505_description')
    me = get_package_share_directory('ame505_bringup')
    peripherals = hiwonder_pkg('peripherals')
    controller = hiwonder_pkg('controller')
    robot_controller = hiwonder_pkg('ros_robot_controller')
    servo = hiwonder_pkg('servo_controller')

    use_camera = LaunchConfiguration('use_camera')
    use_teleop = LaunchConfiguration('use_teleop')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(desc, 'urdf', 'jetauto.xacro')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('astra_bridge', default_value=os.environ.get(
            'AME505_ASTRA_BRIDGE',
            os.path.expanduser('~/jetauto_toolkit/scripts/camera/astra_bridge.py'))),

        # --- URDF / TF -----------------------------------------------------
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher',
             parameters=[{'source_list': ['/controller_manager/joint_states'],
                          'rate': 5.0}]),   # wheel joints are only cosmetic; 20 Hz cost ~17% CPU

        # --- Base: STM32 board, wheel odom, IMU filter, EKF -----------------
        include(robot_controller, 'launch/ros_robot_controller.launch.py',
                {'imu_frame': 'imu_link'}),
        Node(package='controller', executable='odom_publisher', name='odom_publisher',
             output='screen',
             parameters=[os.path.join(controller, 'config/calibrate_params.yaml'),
                         {'base_frame_id': 'base_footprint', 'odom_frame_id': 'odom',
                          'pub_odom_topic': True}]),
        include(peripherals, 'launch/imu_filter.launch.py'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen',
             parameters=[ReplaceString(source_file=os.path.join(controller, 'config/ekf.yaml'),
                                       replacements={'namespace/': ''})],
             remappings=[('odometry/filtered', 'odom'), ('cmd_vel', 'controller/cmd_vel')]),
        include(servo, 'launch/servo_controller.launch.py', {'base_frame': 'base_footprint'}),
        include(controller, 'launch/init_pose.launch.py',
                {'namespace': '', 'use_namespace': 'false', 'action_name': 'horizontal'}),

        # --- Sensors ---------------------------------------------------------
        *lidar(peripherals),
        ExecuteProcess(cmd=['python3', LaunchConfiguration('astra_bridge')],
                       name='astra_bridge', output='screen', sigterm_timeout='3',
                       condition=IfCondition(use_camera)),

        # --- Xbox teleop -------------------------------------------------------
        include(me, 'launch/teleop.launch.py', condition=IfCondition(use_teleop)),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(me, 'rviz', 'robot.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
