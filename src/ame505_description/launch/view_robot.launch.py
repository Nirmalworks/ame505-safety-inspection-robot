"""View the JetAuto URDF alone in RViz (no hardware needed).

    ros2 launch ame505_description view_robot.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('ame505_description')
    use_gui = LaunchConfiguration('use_gui')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'jetauto.xacro')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Show joint_state_publisher_gui sliders'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(use_gui)),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg, 'rviz', 'view_robot.rviz')]),
    ])
