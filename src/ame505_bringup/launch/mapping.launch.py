"""Build a map with RTAB-Map while driving with the Xbox controller.

    ros2 launch ame505_bringup mapping.launch.py map:=lab_map            # resume / extend
    ros2 launch ame505_bringup mapping.launch.py map:=lab_map new:=true  # start from scratch

The database is saved continuously to $AME505_DATA/maps/<map>.db (SSD).
Ctrl+C when done - nothing else is needed to "save" the map.
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
    new = LaunchConfiguration('new').perform(context).lower() == 'true'
    use_camera = LaunchConfiguration('use_camera')
    db = map_db(map_name)
    exists = os.path.exists(db)

    params = base_params(db, use_camera.perform(context) == 'true',
                         LaunchConfiguration('map_3d').perform(context) == 'true')
    params.update({'Mem/IncrementalMemory': 'true', 'Mem/InitWMWithAllNodes': 'false'})
    args = ['-d'] if new else []  # -d deletes the existing database first

    msg = (f'[ame505] MAPPING "{map_name}" -> {db} '
           + ('(NEW: old database deleted)' if new else
              '(appending to existing map)' if exists else '(new database)'))
    return [
        LogInfo(msg=msg),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(me, 'launch', 'robot.launch.py')),
            launch_arguments={'use_camera': LaunchConfiguration('use_camera'),
                              'use_teleop': LaunchConfiguration('use_teleop')}.items()),
        Node(package='ame505_bringup', executable='frame_recorder', name='frame_recorder',
             output='screen', parameters=[{'map': map_name, 'new': new}],
             condition=IfCondition(use_camera)),
        TimerAction(period=8.0, actions=rtabmap_nodes(
            params, args, LaunchConfiguration('rtabmap_viz'))),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(me, 'rviz', 'mapping.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='lab_map', description='map name'),
        DeclareLaunchArgument('new', default_value='false', description='delete old db first'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='false = lidar-only RTAB-Map (lighter, no 3D)'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),   # RViz exhausts the Nano's RAM
        DeclareLaunchArgument('map_3d', default_value='true',
                              description='build coloured 3D map from the depth camera'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        OpaqueFunction(function=setup),
    ])
