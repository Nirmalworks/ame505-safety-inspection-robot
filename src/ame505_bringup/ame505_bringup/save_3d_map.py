#!/usr/bin/env python3
"""Save RTAB-Map's coloured 3D map (/cloud_map) as a .ply point cloud.

Run while mapping.launch.py (or navigation.launch.py) is running:
    ros2 run ame505_bringup save_3d_map --name lab_map
-> $AME505_DATA/maps_3d/lab_map_<date>.ply   (open with MeshLab / CloudCompare)
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from ame505_bringup.paths import data_dir


def write_ply(path, xyz, rgb):
    n = len(xyz)
    header = ('ply\nformat binary_little_endian 1.0\n'
              f'element vertex {n}\n'
              'property float x\nproperty float y\nproperty float z\n'
              + ('property uchar red\nproperty uchar green\nproperty uchar blue\n' if rgb is not None else '')
              + 'end_header\n')
    if rgb is not None:
        dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
        arr = np.empty(n, dtype=dt)
        arr['r'], arr['g'], arr['b'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    else:
        dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
        arr = np.empty(n, dtype=dt)
    arr['x'], arr['y'], arr['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(arr.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', default='lab_map')
    ap.add_argument('--topic', default='/cloud_map')
    ap.add_argument('--timeout', type=float, default=30.0)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node('save_3d_map')
    got = []
    node.create_subscription(PointCloud2, args.topic, got.append, 1)
    node.get_logger().info(f'Waiting for {args.topic} (move the robot a little if it takes long)...')
    t0 = time.time()
    while not got and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not got:
        node.get_logger().error(f'No {args.topic} received. Is mapping running with map_3d:=true?')
        sys.exit(1)
    msg = got[0]
    names = [f.name for f in msg.fields]
    has_rgb = 'rgb' in names
    fields = ['x', 'y', 'z'] + (['rgb'] if has_rgb else [])
    pts = point_cloud2.read_points_numpy(msg, field_names=fields, skip_nans=True) if not has_rgb else None
    if has_rgb:
        st = point_cloud2.read_points(msg, field_names=fields, skip_nans=True)
        xyz = np.stack([st['x'], st['y'], st['z']], axis=1).astype(np.float32)
        packed = np.asarray(st['rgb'], dtype=np.float32).view(np.uint32)
        rgb = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], axis=1).astype(np.uint8)
    else:
        xyz, rgb = pts.astype(np.float32), None
    out = os.path.join(data_dir('maps_3d'), f'{args.name}_{time.strftime("%Y%m%d_%H%M%S")}.ply')
    write_ply(out, xyz, rgb)
    node.get_logger().info(f'Saved {len(xyz)} points -> {out}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
