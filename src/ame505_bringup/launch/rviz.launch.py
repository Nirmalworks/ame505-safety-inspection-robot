"""RViz for mapping or navigation. Run on a laptop (same ROS_DOMAIN_ID, same
Wi-Fi) to spare the Jetson Nano's RAM, or on the robot with a display.

    ros2 launch ame505_bringup rviz.launch.py mode:=robot        # just driving (odom frame)
    ros2 launch ame505_bringup rviz.launch.py mode:=mapping
    ros2 launch ame505_bringup rviz.launch.py mode:=navigation
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    cfg = os.path.join(get_package_share_directory('ame505_bringup'), 'rviz', f'{mode}.rviz')
    return [Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                 arguments=['-d', cfg])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='navigation',
                              description='robot | mapping | navigation'),
        OpaqueFunction(function=setup),
    ])
