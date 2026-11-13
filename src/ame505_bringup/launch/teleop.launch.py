"""Xbox controller (USB or Bluetooth) -> /joy -> xbox_teleop -> /controller/cmd_vel.

    ros2 launch ame505_bringup teleop.launch.py

Bluetooth: pair the controller once on the robot's host desktop (Bluetooth settings).
It then shows up here automatically; `source scripts/env.sh` fixes /dev permissions.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

SDL_FIX = ('050000005e040000130b000009050000,Xbox Series X Controller,a:b0,b:b1,x:b3,y:b4,'
           'leftshoulder:b6,rightshoulder:b7,back:b10,start:b11,guide:b12,leftstick:b13,'
           'rightstick:b14,leftx:a0,lefty:a1,rightx:a2,righty:a3,lefttrigger:a5,righttrigger:a4,'
           'dpup:h0.1,dpright:h0.2,dpdown:h0.4,dpleft:h0.8,misc1:b15,platform:Linux,')


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('ame505_bringup'), 'config', 'xbox.yaml')
    return LaunchDescription([
        SetEnvironmentVariable('SDL_GAMECONTROLLERCONFIG',
                               os.environ.get('SDL_GAMECONTROLLERCONFIG', SDL_FIX)),
        # game_controller_node = SDL GameController layout (6 axes / 21 buttons, LB=9).
        # joy_node would give the RAW layout (8 axes / 16 buttons, LB=6) -> wrong buttons.
        Node(package='joy', executable='game_controller_node', name='joy_node', output='screen',
             parameters=[cfg]),
        Node(package='ame505_bringup', executable='xbox_teleop', name='xbox_teleop',
             output='screen', parameters=[cfg]),
    ])
