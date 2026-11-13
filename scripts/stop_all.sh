#!/usr/bin/env bash
# Stop all ROS 2 launches/nodes started from this container (ours or Hiwonder's),
# then zero the wheels on the motor board (it keeps the last speed otherwise).
source "$(dirname "$0")/env.sh" >/dev/null 2>&1
ros2 topic pub -t 10 -r 20 /controller/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 &
pkill -INT -f "ros2 launch" || true
sleep 3
PAT="/opt/ros/humble/lib/|/ros2_ws/install/|/ame505-safety-inspection-robot/install/ame505_bringup/lib/ame505_bringup/(xbox_teleop|checkpoint_recorder|patrol|depth_obstacles|gtsam_slam|dashboard|frame_recorder|lifecycle_guard)|/third_party_ros2/|astra_bridge.py"
pkill -f "$PAT" || true
# A busy Nav2 container (or Hiwonder's odom_publisher) can ignore SIGTERM and survive as an
# orphan; the next launch then runs TWO Nav2 stacks fighting over the same topics. Escalate.
for i in 1 2 3 4 5; do pgrep -f "$PAT" >/dev/null || break; sleep 1; done
pkill -9 -f "$PAT" 2>/dev/null && echo "force-killed leftovers"
pkill -9 -f "ros2 launch ame505_bringup" 2>/dev/null
sleep 1
ros2 run ame505_bringup motor_stop
ros2 daemon stop >/dev/null 2>&1 || true
echo "stopped"
