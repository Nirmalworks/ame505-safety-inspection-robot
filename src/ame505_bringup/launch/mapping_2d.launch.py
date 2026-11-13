"""2D lidar mapping with slam_toolbox while driving with the Xbox controller (hold LB).

    ros2 launch ame505_bringup mapping_2d.launch.py map:=lab2d
    ./scripts/save_2d_map.sh lab2d          # while it runs: writes maps_2d/lab2d.pgm/.yaml

The dashboard's "Stop & Save" does the saving for you. The map frame = where the robot was
when mapping started, so navigation later assumes the robot starts at that same spot
(or at the "home" you set) unless you use "Set pose".

Lidar only by default: on the Jetson Nano the camera bridge costs CPU that slam_toolbox
needs, and the 2D map does not use the camera. use_camera:=true keeps the live camera view.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    me = get_package_share_directory('ame505_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='lab2d',
                              description='name used by save_2d_map.sh / the dashboard'),
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        LogInfo(msg=['[ame505] 2D MAPPING (slam_toolbox, lidar) -> map "',
                     LaunchConfiguration('map'), '"']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(me, 'launch', 'robot.launch.py')),
            launch_arguments={'use_camera': LaunchConfiguration('use_camera'),
                              'use_teleop': LaunchConfiguration('use_teleop')}.items()),
        # wait for lidar + EKF odom -> base_footprint before slam_toolbox starts
        TimerAction(period=6.0, actions=[Node(
            package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(me, 'config', 'slam_toolbox_mapping.yaml')])]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(me, 'rviz', 'mapping.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
