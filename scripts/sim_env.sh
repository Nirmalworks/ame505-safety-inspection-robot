# Source this on a LAPTOP to run the stack in simulation (no robot, no Gazebo):
#     source scripts/sim_env.sh
# Needs ROS 2 Humble with Nav2 + slam_toolbox, and the workspace built (colcon build --symlink-install).
# Then every normal command works against the simulated robot, e.g.
#     ros2 launch ame505_bringup navigation_2d.launch.py map:=sim_hall use_rviz:=true
#     ros2 run ame505_bringup patrol --map sim_hall
export AME505_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
export AME505_SIM=1                                   # robot.launch.py starts sim_robot, not the hardware
export AME505_DATA="${AME505_DATA:-$HOME/ame505_sim_data}"
# Stay off the network: on the lab Wi-Fi the simulation's /cmd_vel would otherwise reach the REAL robot.
export ROS_LOCALHOST_ONLY=1
# The URDF reads these (on the robot Hiwonder's ~/ros2_ws/.robotrc sets them). A1 lidar as in the
# docs; machine and camera names are assumed. Any camera value other than None adds the camera link.
export MACHINE_TYPE="${MACHINE_TYPE:-JetAuto}" LIDAR_TYPE="${LIDAR_TYPE:-A1}" DEPTH_CAMERA_TYPE="${DEPTH_CAMERA_TYPE:-AstraProPlus}"

if [ -n "$ZSH_VERSION" ]; then _ext=zsh; else _ext=bash; fi
source "/opt/ros/humble/setup.$_ext"
[ -f "$AME505_ROOT/install/local_setup.$_ext" ] && source "$AME505_ROOT/install/local_setup.$_ext"
unset _ext

# First use: put the sample map and patrol route where the stack looks for them.
mkdir -p "$AME505_DATA/maps_2d" "$AME505_DATA/checkpoints"
cp -n "$AME505_ROOT/src/ame505_bringup/sim/sim_hall.pgm" "$AME505_ROOT/src/ame505_bringup/sim/sim_hall.yaml" "$AME505_DATA/maps_2d/"
cp -n "$AME505_ROOT/src/ame505_bringup/sim/sim_hall_checkpoints.yaml" "$AME505_DATA/checkpoints/sim_hall.yaml"
true
