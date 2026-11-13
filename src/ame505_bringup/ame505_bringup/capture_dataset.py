#!/usr/bin/env python3
"""Capture a labeled burst of RGB-D frames for the AME 505 hazard dataset.

Run while the robot software is running (drive, or navigation so the map pose is recorded):

    ros2 run ame505_bringup capture_dataset --session s1_day1 --checkpoint cp_03 \
        --scene hazard --hazard blocked_exit --objects box --light normal --n 10

Each call saves --n frame pairs, --gap seconds apart (nudge/rotate the robot a little between
calls for variety), to
    $AME505_DATA/dataset/<session>/<checkpoint>/<scene>_<hazard>_<time>_<k>_rgb.png / _depth.png
and appends one row per frame to $AME505_DATA/dataset/index.csv with all labels + the robot
pose (map frame if localized, else odom). Depth is 16-bit PNG in millimetres, aligned to RGB.
The checkpoint-level ground truth (normal / hazard + type) comes from these labels, as the
proposal's staging log; bounding boxes are drawn later in CVAT.
"""
import argparse
import csv
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

from ame505_bringup.paths import data_dir

HAZARDS = ['none', 'blocked_exit', 'obstructed_walkway', 'open_door', 'multiple']
FIELDS = ['file_rgb', 'file_depth', 'stamp', 'session', 'checkpoint', 'scene', 'hazard', 'objects',
          'light', 'occluded', 'note', 'pose_frame', 'x', 'y', 'yaw', 'depth_valid_pct', 'rgb_age_s']


class Capture(Node):
    def __init__(self):
        super().__init__('capture_dataset')
        self.rgb = self.depth = None
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', lambda m: setattr(self, 'rgb', m),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', lambda m: setattr(self, 'depth', m),
                                 qos_profile_sensor_data)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)

    def pose(self):
        for frame in ('map', 'odom'):
            try:
                t = self.tf.lookup_transform(frame, 'base_footprint', Time())
            except Exception:  # noqa: BLE001
                continue
            q, p = t.transform.rotation, t.transform.translation
            return frame, p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return '', float('nan'), float('nan'), float('nan')


def to_np(m):
    buf = np.frombuffer(m.data, dtype=np.uint8)
    if m.encoding in ('bgr8', 'rgb8'):
        img = buf.reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        return img if m.encoding == 'bgr8' else img[:, :, ::-1]
    if m.encoding in ('16UC1', 'mono16'):
        return buf.view(np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
    raise ValueError(f'unsupported encoding {m.encoding}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--session', required=True, help='e.g. s1_day1 (one per day / lighting setup)')
    ap.add_argument('--checkpoint', required=True, help='e.g. cp_01 (same ids as the patrol checkpoints)')
    ap.add_argument('--scene', choices=['normal', 'hazard'], required=True)
    ap.add_argument('--hazard', choices=HAZARDS, default='none')
    ap.add_argument('--objects', default='', help='staged objects, e.g. "box,chair"')
    ap.add_argument('--light', choices=['normal', 'low'], default='normal')
    ap.add_argument('--occluded', choices=['0', '1'], default='0', help='hazard partly hidden')
    ap.add_argument('--note', default='')
    ap.add_argument('--n', type=int, default=10, help='frames in this burst')
    ap.add_argument('--gap', type=float, default=0.7, help='seconds between frames')
    a = ap.parse_args()
    if a.scene == 'normal':
        a.hazard = 'none'
    elif a.hazard == 'none':
        ap.error('--scene hazard needs --hazard')

    rclpy.init()
    node = Capture()
    out = data_dir(os.path.join('dataset', a.session, a.checkpoint))
    index = os.path.join(data_dir('dataset'), 'index.csv')
    new_index = not os.path.exists(index)
    t0 = time.time()
    while (node.rgb is None or node.depth is None) and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.rgb is None or node.depth is None:
        print('No camera images. Is the robot software running WITH the camera '
              '(dashboard "camera" ticked / use_camera:=true)?')
        return 1

    saved = 0
    tag = f'{a.scene}_{a.hazard}_{time.strftime("%H%M%S")}'
    with open(index, 'a', newline='') as f:
        w = csv.DictWriter(f, FIELDS)
        if new_index:
            w.writeheader()
        last = None
        while saved < a.n:
            t_end = time.time() + a.gap
            while time.time() < t_end:
                rclpy.spin_once(node, timeout_sec=0.05)
            if node.rgb is last:                       # camera is slow: wait for a NEW frame
                continue
            last, rgb_m, dep_m = node.rgb, node.rgb, node.depth
            rgb, dep = to_np(rgb_m), to_np(dep_m)
            base = os.path.join(out, f'{tag}_{saved:02d}')
            cv2.imwrite(base + '_rgb.png', rgb)
            cv2.imwrite(base + '_depth.png', dep.astype(np.uint16))
            frame, x, y, yaw = node.pose()
            stamp = rgb_m.header.stamp.sec + rgb_m.header.stamp.nanosec * 1e-9
            w.writerow({'file_rgb': os.path.relpath(base + '_rgb.png', data_dir('dataset')),
                        'file_depth': os.path.relpath(base + '_depth.png', data_dir('dataset')),
                        'stamp': f'{stamp:.3f}', 'session': a.session, 'checkpoint': a.checkpoint,
                        'scene': a.scene, 'hazard': a.hazard, 'objects': a.objects, 'light': a.light,
                        'occluded': a.occluded, 'note': a.note, 'pose_frame': frame,
                        'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3),
                        'depth_valid_pct': round(100.0 * float((dep > 0).mean()), 1),
                        'rgb_age_s': round(node.get_clock().now().nanoseconds * 1e-9 - stamp, 2)})
            f.flush()
            saved += 1
            print(f'  {saved}/{a.n}  {os.path.basename(base)}  ({rgb.shape[1]}x{rgb.shape[0]}, pose {frame})')
    total = sum(1 for _ in open(index)) - 1
    print(f'Saved {saved} frames to {out}. Dataset total: {total} frames ({index}).')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
