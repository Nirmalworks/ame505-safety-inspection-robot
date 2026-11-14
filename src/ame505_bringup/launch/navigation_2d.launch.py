"""Navigate on a saved 2D map: map_server + AMCL (lidar localization) + Nav2.

    ros2 launch ame505_bringup navigation_2d.launch.py map:=lab2d
    ros2 launch ame505_bringup navigation_2d.launch.py map:=lab2d initial_x:=1.0 initial_y:=0.5 initial_yaw:=1.57

Uses maps_2d/<map>.yaml (made by mapping_2d.launch.py / the dashboard). The robot is assumed
to start at the map's "home" (set on the dashboard; default 0,0,0 = where mapping started).
If it starts elsewhere: dashboard "Set pose" (or RViz "2D Pose Estimate"); AMCL then
tracks it with the lidar. Lidar only by default: the depth camera arrives ~5-7 s late over
the ROS 1 bridge, too late for obstacle avoidance while moving. use_camera:=true adds it.
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ame505_bringup.paths import data_dir, map_yaml


def setup(context):
    me = get_package_share_directory('ame505_bringup')
    name = LaunchConfiguration('map').perform(context)
    yml = map_yaml(name)
    if not os.path.exists(yml):
        raise RuntimeError(f'No 2D map at {yml}. Make one with mapping_2d.launch.py map:={name}.')

    pose = {k: LaunchConfiguration(f'initial_{k}').perform(context) for k in ('x', 'y', 'yaw')}
    home = os.path.join(data_dir('home'), f'{name}.yaml')
    if all(v == '' for v in pose.values()):              # not given -> the map's home
        h = (yaml.safe_load(open(home)) or {}) if os.path.exists(home) else {}
        pose = {k: str(h.get(k, 0.0)) for k in ('x', 'y', 'yaw')}
    use_camera = LaunchConfiguration('use_camera')

    return [
        LogInfo(msg=f'[ame505] 2D NAVIGATION on "{name}" ({yml}), AMCL start pose '
                    f'x={pose["x"]} y={pose["y"]} yaw={pose["yaw"]}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(me, 'launch', 'robot.launch.py')),
            launch_arguments={'use_camera': use_camera,
                              'use_teleop': LaunchConfiguration('use_teleop')}.items()),
        Node(package='ame505_bringup', executable='depth_obstacles', name='depth_obstacles',
             output='screen', condition=IfCondition(use_camera)),
        # keep-clear check at each patrol checkpoint (needs the camera)
        Node(package='ame505_bringup', executable='inspection', name='inspection',
             output='screen', condition=IfCondition(use_camera)),
        TimerAction(period=6.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(me, 'launch', 'nav2_stack.launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file').perform(context),
                'map_yaml': yml,
                'initial_x': pose['x'], 'initial_y': pose['y'], 'initial_yaw': pose['yaw'],
            }.items())]),
        # finishes the map_server/AMCL bringup if Nav2's lifecycle manager gives up under load
        Node(package='ame505_bringup', executable='lifecycle_guard', name='lifecycle_guard',
             output='screen'),
        Node(package='ame505_bringup', executable='checkpoint_recorder',
             name='checkpoint_recorder', output='screen',
             parameters=[os.path.join(me, 'config', 'xbox.yaml'), {'map': name}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(me, 'rviz', 'navigation.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    me = get_package_share_directory('ame505_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='lab2d'),
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value=''),
        DeclareLaunchArgument('initial_y', default_value=''),
        DeclareLaunchArgument('initial_yaw', default_value=''),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(me, 'config', 'nav2_params.yaml')),
        OpaqueFunction(function=setup),
    ])
