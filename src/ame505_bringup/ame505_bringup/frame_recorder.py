#!/usr/bin/env python3
"""Record every useful RGB-D frame during mapping, for dense 3D reconstruction.

RTAB-Map keeps at most ~1 node/s. For a realistic scene we want many more views, so this
saves each camera frame (colour JPEG + depth PNG) whenever the robot has moved a little,
with its ODOMETRY pose (odom -> base_footprint) at the frame's timestamp.
reconstruct_3d later corrects those poses with RTAB-Map's optimized (loop-closed) graph.

Output: $AME505_DATA/frames/<map>/  frames.csv, camera.yaml, rgb/*.jpg, depth/*.png
"""
import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from ame505_bringup.paths import data_dir


def tf_to_T(t):
    q, p = t.transform.rotation, t.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [p.x, p.y, p.z]
    return T


class FrameRecorder(Node):
    def __init__(self):
        super().__init__('frame_recorder')
        name = self.declare_parameter('map', 'lab_map').value
        self.min_dist = self.declare_parameter('min_dist', 0.08).value      # m
        self.min_angle = self.declare_parameter('min_angle', 0.08).value    # rad (~5 deg)
        self.min_dt = self.declare_parameter('min_dt', 0.25).value          # s
        new = self.declare_parameter('new', False).value
        self.dir = data_dir(os.path.join('frames', name))
        os.makedirs(os.path.join(self.dir, 'rgb'), exist_ok=True)
        os.makedirs(os.path.join(self.dir, 'depth'), exist_ok=True)
        self.csv_path = os.path.join(self.dir, 'frames.csv')
        if new and os.path.exists(self.csv_path):
            for sub in ('rgb', 'depth'):
                for f in os.listdir(os.path.join(self.dir, sub)):
                    os.remove(os.path.join(self.dir, sub, f))
            os.remove(self.csv_path)
        self.count = sum(1 for _ in open(self.csv_path)) - 1 if os.path.exists(self.csv_path) else 0
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w') as f:
                f.write('idx,stamp,' + ','.join(f'o{i}' for i in range(12)) + '\n')

        self.tf = Buffer(cache_time=Duration(seconds=30))
        TransformListener(self.tf, self)
        self.cam_saved = os.path.exists(os.path.join(self.dir, 'camera.yaml'))
        self.info = None
        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.on_info, qos_profile_sensor_data)
        rgb = Subscriber(self, Image, '/depth_cam/rgb/image_raw', qos_profile=qos_profile_sensor_data)
        dep = Subscriber(self, Image, '/depth_cam/depth/image_raw', qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([rgb, dep], 10, 0.05)
        self.sync.registerCallback(self.on_frame)
        self.last = None
        self.get_logger().info(f'Recording RGB-D frames to {self.dir} ({self.count} already there)')

    def on_info(self, m):
        self.info = m

    def save_camera(self, rgb_msg):
        if self.cam_saved or self.info is None:
            return
        try:
            t = self.tf.lookup_transform('base_footprint', rgb_msg.header.frame_id, Time())
        except Exception:  # noqa: BLE001
            return
        k = [float(v) for v in self.info.k]          # numpy float64 -> plain float for YAML
        with open(os.path.join(self.dir, 'camera.yaml'), 'w') as f:
            yaml.safe_dump({'width': int(self.info.width), 'height': int(self.info.height),
                            'fx': k[0], 'fy': k[4], 'cx': k[2], 'cy': k[5],
                            'base_to_camera': [float(v) for v in tf_to_T(t)[:3, :4].ravel()],
                            'frame_id': rgb_msg.header.frame_id}, f)
        self.cam_saved = True
        self.get_logger().info('camera intrinsics + mounting saved')

    def on_frame(self, rgb_msg, dep_msg):
        self.save_camera(rgb_msg)
        if not self.cam_saved:
            return
        stamp = Time.from_msg(rgb_msg.header.stamp)
        try:
            t = self.tf.lookup_transform('odom', 'base_footprint', stamp, timeout=Duration(seconds=0.2))
        except Exception:  # noqa: BLE001
            return
        T = tf_to_T(t)
        s = stamp.nanoseconds * 1e-9
        if self.last is not None:
            Tl, sl = self.last
            d = np.linalg.norm(T[:2, 3] - Tl[:2, 3])
            a = abs(math.atan2(T[1, 0], T[0, 0]) - math.atan2(Tl[1, 0], Tl[0, 0]))
            a = min(a, 2 * math.pi - a)
            if s - sl < self.min_dt or (d < self.min_dist and a < self.min_angle):
                return
        h, w = rgb_msg.height, rgb_msg.width
        rgb = np.frombuffer(rgb_msg.data, np.uint8).reshape(h, rgb_msg.step)[:, :w * 3].reshape(h, w, 3)
        if rgb_msg.encoding == 'rgb8':
            rgb = rgb[:, :, ::-1]
        dep = np.frombuffer(dep_msg.data, np.uint16).reshape(dep_msg.height, dep_msg.step // 2)[:, :dep_msg.width]
        i = self.count
        cv2.imwrite(os.path.join(self.dir, 'rgb', f'{i:06d}.jpg'), rgb, [cv2.IMWRITE_JPEG_QUALITY, 92])
        cv2.imwrite(os.path.join(self.dir, 'depth', f'{i:06d}.png'), dep)
        with open(self.csv_path, 'a') as f:
            f.write(f'{i},{s:.6f},' + ','.join(f'{v:.6f}' for v in T[:3, :4].ravel()) + '\n')
        self.count += 1
        self.last = (T, s)
        if self.count % 25 == 0:
            self.get_logger().info(f'{self.count} frames recorded')


def main():
    rclpy.init()
    node = FrameRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
