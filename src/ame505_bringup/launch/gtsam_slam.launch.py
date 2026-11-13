"""GTSAM lidar/gyro odometry next to the running robot software (read-only, never drives).

    ros2 launch ame505_bringup gtsam_slam.launch.py                 # lidar + gyro
    ros2 launch ame505_bringup gtsam_slam.launch.py use_imu:=false  # lidar only

Start it while drive / mapping / navigation is running and keep the robot STILL ~4 s.
Outputs /gtsam/odom, /gtsam/path, /gtsam/status; TF map (or odom) -> gtsam_odom.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('ame505_bringup'), 'config', 'gtsam_slam.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_imu', default_value='true'),
        Node(package='ame505_bringup', executable='gtsam_slam', name='gtsam_slam', output='screen',
             parameters=[cfg, {'use_imu': ParameterValue(LaunchConfiguration('use_imu'), value_type=bool)}]),
    ])
