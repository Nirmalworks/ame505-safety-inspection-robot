"""Localize on a saved RTAB-Map map and run Nav2.

    ros2 launch ame505_bringup navigation.launch.py map:=lab_map

Then send goals with RViz "2D Goal Pose", record checkpoints with Y on the
Xbox controller, and run the patrol:  ros2 run ame505_bringup patrol --map lab_map
The Xbox teleop stays available (hold LB) to override / reposition the robot.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ame505_bringup.paths import map_db
from ame505_bringup.rtabmap_config import base_params, rtabmap_nodes


def setup(context):
    me = get_package_share_directory('ame505_bringup')
    map_name = LaunchConfiguration('map').perform(context)
    db = map_db(map_name)
    if not os.path.exists(db):
        raise RuntimeError(f'No map database at {db}. Run mapping.launch.py map:={map_name} first.')

    use_camera = LaunchConfiguration('use_camera')
    params = base_params(db, use_camera.perform(context) == 'true',
                         LaunchConfiguration('map_3d').perform(context) == 'true')
    # Localization mode: read-only map, load whole graph into working memory
    params.update({'Mem/IncrementalMemory': 'false', 'Mem/InitWMWithAllNodes': 'true',
                   # Assume the robot starts where mapping started (map origin) so it appears on
                   # the map right away; RTAB-Map then corrects it when it recognises a place.
                   # Started somewhere else? Use "Set pose" on the dashboard map.
                   'RGBD/StartAtOrigin': 'true'})

    cfg = os.path.join(me, 'config', 'xbox.yaml')

    return [
        LogInfo(msg=f'[ame505] NAVIGATION on "{map_name}" ({db})'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(me, 'launch', 'robot.launch.py')),
            launch_arguments={'use_camera': use_camera,
                              'use_teleop': LaunchConfiguration('use_teleop')}.items()),
        TimerAction(period=8.0, actions=rtabmap_nodes(
            params, [], LaunchConfiguration('rtabmap_viz'))),
        # Live depth-camera obstacles for the costmaps + collision monitor
        Node(package='ame505_bringup', executable='depth_obstacles', name='depth_obstacles',
             output='screen', condition=IfCondition(use_camera)),
        # Nav2 with OUR params: holonomic DWB, lidar+depth costmaps where live data overrides
        # the saved map, no spin/back-up recoveries, collision monitor before the motors.
        TimerAction(period=12.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(me, 'launch', 'nav2_stack.launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file').perform(context),
            }.items())]),
        Node(package='ame505_bringup', executable='checkpoint_recorder',
             name='checkpoint_recorder', output='screen',
             parameters=[cfg, {'map': map_name}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(me, 'rviz', 'navigation.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    me = get_package_share_directory('ame505_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='lab_map'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('map_3d', default_value='true',
                              description='build coloured 3D map from the depth camera'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(me, 'config', 'nav2_params.yaml')),
        OpaqueFunction(function=setup),
    ])
