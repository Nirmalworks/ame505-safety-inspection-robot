#!/usr/bin/env python3
"""Inspect each patrol checkpoint: take one RGB-D frame and decide normal / hazard.

    ros2 run ame505_bringup inspection          # next to navigation WITH the camera + the patrol

The patrol publishes the checkpoint id on patrol/at_checkpoint when it arrives. This node then
waits `settle_s` (camera frames reach ROS 2 a few seconds late, so the first ones still show the
robot driving), takes the next NEW depth + RGB frame and runs the keep-clear depth rule
(keep_clear.py, zones in config/inspection.yaml). Each inspection is

  * saved:      $AME505_DATA/inspections/<date>/<time>_<checkpoint>_rgb.png / _depth.png
  * logged:     $AME505_DATA/inspections/index.csv
  * published:  inspection/result (std_msgs/String, JSON)

The patrol's --dwell must be longer than settle_s plus one camera frame (default dwell 5 s is enough).
The trained detector will be added here as a second opinion once it exists.
"""
import csv
import json
import os
import time

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from ame505_bringup.capture_dataset import to_np
from ame505_bringup.keep_clear import check_checkpoint, load_config
from ame505_bringup.paths import data_dir

FIELDS = ['time', 'checkpoint', 'result', 'area_m2', 'points', 'nearest_m', 'valid_pct', 'file_rgb', 'file_depth']


class Inspection(Node):
    def __init__(self):
        super().__init__('inspection')
        default = os.path.join(get_package_share_directory('ame505_bringup'), 'config', 'inspection.yaml')
        self.config = load_config(self.declare_parameter('config', default).value)
        self.settle = self.declare_parameter('settle_s', 3.0).value
        self.timeout = self.declare_parameter('timeout_s', 12.0).value
        self.rgb = self.depth = None            # (message, arrival time)
        self.camera = None                      # intrinsics from camera_info, when published
        self.pending = None                     # (checkpoint, time it was reached)
        self.create_subscription(Image, '/depth_cam/rgb/image_raw',
                                 lambda m: setattr(self, 'rgb', (m, time.monotonic())), qos_profile_sensor_data)
        self.create_subscription(Image, '/depth_cam/depth/image_raw',
                                 lambda m: setattr(self, 'depth', (m, time.monotonic())), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.on_info, 5)
        self.create_subscription(String, 'patrol/at_checkpoint', self.on_checkpoint, 10)
        self.result_pub = self.create_publisher(String, 'inspection/result', 10)
        self.create_timer(0.2, self.tick)
        self.get_logger().info(f'Inspection ready: keep-clear zone {self.config.get("zone")}, '
                               f'hazard at >= {self.config.get("min_area_m2")} m2')

    def on_info(self, m):
        if m.k[0] > 0:
            self.camera = {'fx': m.k[0], 'fy': m.k[4], 'cx': m.k[2], 'cy': m.k[5]}

    def on_checkpoint(self, msg):
        self.pending = (msg.data, time.monotonic())

    def tick(self):
        if self.pending is None:
            return
        cp, reached = self.pending
        now = time.monotonic()
        fresh = reached + self.settle
        if self.depth and self.rgb and self.depth[1] >= fresh and self.rgb[1] >= fresh:
            self.pending = None
            self.inspect(cp, to_np(self.rgb[0]), to_np(self.depth[0]))
        elif now - reached > self.timeout:
            self.pending = None
            self.get_logger().error(f'{cp}: no camera image within {self.timeout:.0f} s (camera on?)')
            self.record({'checkpoint': cp, 'result': 'no_image'})

    def inspect(self, cp, rgb, depth):
        res = check_checkpoint(depth, self.config, cp, self.camera)
        day = data_dir(os.path.join('inspections', time.strftime('%Y%m%d')))
        base = os.path.join(day, f'{time.strftime("%H%M%S")}_{cp}')
        cv2.imwrite(base + '_rgb.png', rgb)
        cv2.imwrite(base + '_depth.png', depth)
        root = data_dir('inspections')
        self.record({'checkpoint': cp, 'result': 'hazard' if res['hazard'] else 'normal',
                     **{k: res[k] for k in ('area_m2', 'points', 'nearest_m', 'valid_pct')},
                     'file_rgb': os.path.relpath(base + '_rgb.png', root),
                     'file_depth': os.path.relpath(base + '_depth.png', root)})

    def record(self, row):
        row = {'time': time.strftime('%Y-%m-%d %H:%M:%S'), **row}
        index = os.path.join(data_dir('inspections'), 'index.csv')
        new = not os.path.exists(index)
        with open(index, 'a', newline='') as f:
            w = csv.DictWriter(f, FIELDS)
            if new:
                w.writeheader()
            w.writerow(row)
        self.result_pub.publish(String(data=json.dumps(row)))
        text = f'{row["checkpoint"]}: {row["result"].upper()}' + (
            f' ({row["area_m2"]} m2 in the keep-clear zone, nearest {row["nearest_m"]} m)'
            if row.get('area_m2') else '')
        if row['result'] == 'normal':          # (rclpy refuses two severities from one call site)
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)


def main():
    rclpy.init()
    node = Inspection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
