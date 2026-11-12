"""Run the hardware estimator on a bag without ROS or motor access."""

import argparse
from collections import Counter, deque
import csv
from dataclasses import fields
import json
import math
from pathlib import Path
import time

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation
import yaml

from bag_tools import bag_path, stamp_ns
from hardware_core import Config, HardwareOdometry, rotation_matrix, scan_points


def static_frames(reader):
    adjacency = {}
    connections = [c for c in reader.connections if c.topic == '/tf_static']
    if not connections:
        return adjacency
    for connection, _, raw in reader.messages(connections=connections):
        for tr in reader.deserialize(raw, connection.msgtype).transforms:
            matrix = np.eye(4)
            p, q = tr.transform.translation, tr.transform.rotation
            matrix[:3, :3] = rotation_matrix([q.x, q.y, q.z, q.w])
            matrix[:3, 3] = [p.x, p.y, p.z]
            parent, child = tr.header.frame_id, tr.child_frame_id
            adjacency.setdefault(parent, {})[child] = matrix
            adjacency.setdefault(child, {})[parent] = np.linalg.inv(matrix)
    return adjacency


def lookup(adjacency, base, sensor):
    pending, visited = deque([(base, np.eye(4))]), set()
    while pending:
        frame, transform = pending.popleft()
        if frame == sensor:
            return transform[:3, 3], Rotation.from_matrix(transform[:3, :3]).as_quat()
        if frame in visited:
            continue
        visited.add(frame)
        for neighbor, relative in adjacency.get(frame, {}).items():
            pending.append((neighbor, transform @ relative))
    raise ValueError(f'No static TF {base} <- {sensor}; record /tf_static or provide explicit extrinsics')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('jetauto.yaml'))
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--lidar-only', action='store_true')
    parser.add_argument('--lidar-transform', nargs=7, type=float,
                        metavar=('X', 'Y', 'Z', 'QX', 'QY', 'QZ', 'QW'),
                        help='Explicit base<-lidar override when TF is unavailable')
    parser.add_argument('--imu-rotation', nargs=4, type=float, metavar=('QX', 'QY', 'QZ', 'QW'),
                        help='Explicit base<-IMU quaternion override')
    args = parser.parse_args()
    settings = yaml.safe_load(args.config.read_text())['jetauto_gtsam']['ros__parameters']
    config = Config(**{f.name: settings[f.name] for f in fields(Config) if f.name in settings})
    if args.lidar_only:
        config.use_imu = False
    estimator = HardwareOdometry(config)
    args.out.mkdir(parents=True, exist_ok=True)
    states, rows, processing = Counter(), [], []
    with AnyReader([bag_path(args.bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        adjacency = static_frames(reader)
        samples = []
        if config.use_imu:
            connections = [c for c in reader.connections if c.topic == settings['imu_topic']]
            if not connections:
                raise ValueError('No IMU topic; select the right topic or use --lidar-only')
            for connection, _, raw in reader.messages(connections=connections):
                msg = reader.deserialize(raw, connection.msgtype)
                if msg.angular_velocity_covariance[0] < 0:
                    raise ValueError('IMU angular velocity marked unavailable')
                quaternion = (args.imu_rotation if args.imu_rotation else
                              lookup(adjacency, settings['base_frame'], msg.header.frame_id)[1])
                w = msg.angular_velocity
                rate = (rotation_matrix(quaternion) @ [w.x, w.y, w.z])[2]
                samples.append((stamp_ns(msg) / 1e9, float(rate)))
            if not samples:
                raise ValueError('No IMU samples; select the right topic or use --lidar-only')
        index, last_scan = 0, -math.inf
        connections = [c for c in reader.connections if c.topic == settings['scan_topic']]
        if not connections:
            raise ValueError(f'No scan messages on {settings["scan_topic"]}')
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            ns = stamp_ns(msg)
            stamp = ns / 1e9
            if 0 < stamp - last_scan < 1 / settings['process_rate_hz']:
                continue
            last_scan = stamp
            while index < len(samples) and (not estimator.gyro.samples or
                                            estimator.gyro.samples[-1][0] < stamp):
                estimator.gyro.add(*samples[index])
                index += 1
            translation, quaternion = ((args.lidar_transform[:3], args.lidar_transform[3:])
                                       if args.lidar_transform else
                                       lookup(adjacency, settings['base_frame'], msg.header.frame_id))
            started = time.perf_counter()
            points = scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                 msg.range_min, msg.range_max, translation, quaternion, config)
            pose, status = estimator.process(stamp, points)
            elapsed = (time.perf_counter() - started) * 1000
            processing.append(elapsed)
            states[status['state']] += 1
            if pose is not None:
                rows.append((ns, pose.x(), pose.y(), pose.theta(), estimator.bias,
                             status.get('overlap'), status.get('rmse_m'), elapsed))
    with (args.out / 'trajectory.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('stamp_ns', 'x', 'y', 'yaw', 'gyro_bias_rad_s',
                         'overlap', 'rmse_m', 'processing_ms'))
        writer.writerows(rows)
    summary = {'bag': str(args.bag), 'imu_enabled': config.use_imu,
               'accepted_poses': len(rows), 'states': dict(states),
               'processing_p95_ms': float(np.percentile(processing, 95)) if processing else None,
               'explicit_lidar_transform': args.lidar_transform,
               'explicit_imu_rotation': args.imu_rotation,
               'note': 'Offline odometry replay; no ground-truth accuracy or live latency claim.'}
    (args.out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    if len(rows) < 2:
        parser.exit(2, 'No usable trajectory; inspect quality states, sensor timing, and extrinsics.\n')


if __name__ == '__main__':
    main()
