#!/usr/bin/env python3
"""Autonomous patrol: visit every recorded checkpoint with Nav2, stop, dwell.

    ros2 run ame505_bringup patrol --map lab_map --loops 1 --dwell 5

At each checkpoint it publishes the checkpoint id on /patrol/at_checkpoint
(std_msgs/String) and waits --dwell seconds. That is the hook for the
inspection node (capture RGB-D -> YOLO -> keep-clear check) from the proposal.
"""
import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String

from ame505_bringup.paths import checkpoints_file


def make_pose(nav, frame, cp):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(cp['x'])
    p.pose.position.y = float(cp['y'])
    p.pose.orientation.z = math.sin(cp['yaw'] / 2.0)
    p.pose.orientation.w = math.cos(cp['yaw'] / 2.0)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='lab_map', help='map name (selects checkpoint file)')
    ap.add_argument('--file', default='', help='explicit checkpoint yaml (overrides --map)')
    ap.add_argument('--loops', type=int, default=1, help='patrol loops (0 = forever)')
    ap.add_argument('--dwell', type=float, default=5.0, help='seconds to stop at each checkpoint')
    ap.add_argument('--return-home', action='store_true', help='go back to cp_01 at the end')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    path = args.file or checkpoints_file(args.map)
    if not os.path.exists(path):
        sys.exit(f'No checkpoint file {path}. Record checkpoints first (Y button during navigation).')
    with open(path) as f:
        data = yaml.safe_load(f)
    cps, frame = data['checkpoints'], data.get('frame_id', 'map')
    if not cps:
        sys.exit(f'No checkpoints in {path}')

    rclpy.init()
    nav = BasicNavigator()
    pub = nav.create_publisher(String, 'patrol/at_checkpoint', 10)
    missed_pub = nav.create_publisher(String, 'patrol/not_reached', 10)   # after two attempts + a later return
    nav.get_logger().info(f'Loaded {len(cps)} checkpoints from {path}')
    # Only wait for Nav2 itself. Naming a localizer makes Humble's BasicNavigator wait for that
    # node's lifecycle service: 'robot_localization' has none (it blocks forever), and 'amcl'
    # re-publishes an initial pose at the origin. Localization is already up by patrol time.
    nav.waitUntilNav2Active(localizer='bt_navigator')

    def drive_to(cp):
        nav.goToPose(make_pose(nav, frame, cp))
        while not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.1)
        return nav.getResult() == TaskResult.SUCCEEDED

    def visit(cp, label):
        """Drive to a checkpoint (one retry) and stop there. False = not reached."""
        nav.get_logger().info(f'[{label}] -> {cp["id"]} ({cp["x"]}, {cp["y"]})')
        if not drive_to(cp):
            nav.get_logger().warn(f'{cp["id"]} not reached, trying once more')
            if not drive_to(cp):
                return False
        pub.publish(String(data=cp['id']))
        nav.get_logger().info(f'Arrived at {cp["id"]}, inspecting for {args.dwell}s')
        time.sleep(args.dwell)
        return True

    loop = 0
    try:
        while args.loops == 0 or loop < args.loops:
            loop += 1
            missed = [cp for cp in cps if not visit(cp, f'loop {loop}')]
            # Something was in the way (people, a cart). Come back at the end of the loop: it may
            # have moved, and the robot now arrives from a different direction.
            missed = [cp for cp in missed if not visit(cp, f'loop {loop}, second attempt')]
            for cp in missed:
                missed_pub.publish(String(data=cp['id']))
            nav.get_logger().info(
                f'Loop {loop}: {len(cps) - len(missed)}/{len(cps)} checkpoints inspected'
                + (f', NOT reached: {", ".join(cp["id"] for cp in missed)}' if missed else ''))
        if args.return_home:
            drive_to(cps[0])
        nav.get_logger().info('Patrol finished')
    except KeyboardInterrupt:
        nav.cancelTask()
        nav.get_logger().warn('Patrol cancelled')
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
