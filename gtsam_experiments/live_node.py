#!/usr/bin/env python3
"""ROS 2 Humble hardware test: lidar ICP + calibrated gyro yaw + GTSAM iSAM2."""

import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from hardware_core import Config, HardwareOdometry, rotation_matrix, scan_points


class JetAutoGtsam(Node):
    def __init__(self, **kwargs):
        super().__init__('jetauto_gtsam', **kwargs)
        defaults = {
            **asdict(Config()), 'scan_topic': '/scan', 'imu_topic': '/imu',
            'base_frame': 'base_footprint', 'output_frame': 'gtsam_odom',
            'process_rate_hz': 5.0, 'sensor_timeout': 1.0,
            'output_dir': 'results/hardware',
        }
        self.declare_parameters('', list(defaults.items()))
        params = {name: self.get_parameter(name).value for name in defaults}
        self.config = Config(**{name: params[name] for name in asdict(Config())})
        if params['process_rate_hz'] <= 0 or params['sensor_timeout'] <= 0:
            raise ValueError('process_rate_hz and sensor_timeout must be positive')
        if not params['base_frame'] or not params['output_frame']:
            raise ValueError('Frame names must not be empty')
        if params['output_frame'] in ('map', 'odom', params['base_frame']):
            raise ValueError('Use a separate output frame, such as gtsam_odom')
        self.settings = params
        self.estimator = HardwareOdometry(self.config)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        imu_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, params['scan_topic'], self.on_scan, scan_qos)
        if self.config.use_imu:
            self.create_subscription(Imu, params['imu_topic'], self.on_imu, imu_qos)
        self.odom_pub = self.create_publisher(Odometry, '/gtsam/odom', 10)
        self.path_pub = self.create_publisher(PathMsg, '/gtsam/path', 1)
        self.status_pub = self.create_publisher(String, '/gtsam/status', 10)
        self.path = PathMsg()
        self.path.header.frame_id = params['output_frame']
        self.pending = None
        self.last_scan_arrival = None
        self.last_imu_arrival = None
        self.imu_tf_error = None
        self.status = {'state': 'waiting_for_scan'}
        session_name = time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns() % 1_000_000_000:09d}'
        output = Path(params['output_dir']).expanduser() / session_name
        output.mkdir(parents=True, exist_ok=False)
        (output / 'parameters.json').write_text(json.dumps(params, indent=2) + '\n')
        self.csv_stream = (output / 'trajectory.csv').open('w', newline='')
        self.writer = csv.writer(self.csv_stream)
        self.writer.writerow(('stamp_ns', 'x', 'y', 'yaw', 'gyro_bias_rad_s',
                              'overlap', 'rmse_m', 'processing_ms'))
        self.status_stream = (output / 'status.jsonl').open('w')
        self.create_timer(1 / params['process_rate_hz'], self.process)
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            f'Reading {params["scan_topic"]}, {params["imu_topic"]}; '
            f'output frame {params["output_frame"]}; logs: {output.resolve()}. '
            'Keep robot stationary for initial gyro calibration.')

    def extrinsics(self, frame, stamp):
        if not frame:
            raise ValueError('Sensor header.frame_id is empty')
        tr = self.tf_buffer.lookup_transform(self.settings['base_frame'], frame,
                                              Time.from_msg(stamp)).transform
        p, q = tr.translation, tr.rotation
        return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)

    def on_imu(self, msg):
        try:
            if msg.angular_velocity_covariance[0] < 0:
                raise ValueError('IMU angular velocity marked unavailable')
            _, quaternion = self.extrinsics(msg.header.frame_id, msg.header.stamp)
            w = msg.angular_velocity
            rate = (rotation_matrix(quaternion) @ [w.x, w.y, w.z])[2]
            stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
            self.estimator.gyro.add(stamp, float(rate))
            self.last_imu_arrival = time.monotonic()
            self.imu_tf_error = None
        except (TransformException, ValueError) as exc:
            self.imu_tf_error = str(exc)

    def on_scan(self, msg):
        self.pending = msg  # Keep the newest scan instead of accumulating latency.
        self.last_scan_arrival = time.monotonic()

    def process(self):
        if self.pending is None:
            return
        # Give the ~50 Hz IMU a short chance to bracket the scan timestamp.
        msg = self.pending
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        stamp = stamp_ns / 1e9
        if self.config.use_imu and not self.imu_tf_error and self.estimator.gyro.samples:
            if self.estimator.gyro.samples[-1][0] < stamp and \
                    time.monotonic() - self.last_scan_arrival < 0.15:
                return
        self.pending = None
        started = time.monotonic()
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age > self.settings['sensor_timeout'] or age < -0.2:
            self.status = {'state': 'scan_clock_mismatch_or_stale', 'age_s': age}
            return
        if self.config.use_imu and self.imu_tf_error:
            self.status = {'state': 'waiting_for_imu_tf', 'detail': self.imu_tf_error}
            return
        try:
            translation, quaternion = self.extrinsics(msg.header.frame_id, msg.header.stamp)
            points = scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                 msg.range_min, msg.range_max, translation, quaternion,
                                 self.config)
            pose, self.status = self.estimator.process(stamp, points)
            if pose is None:
                return
            elapsed = (time.monotonic() - started) * 1000
            self.status['processing_ms'] = elapsed
            self.publish_pose(msg.header.stamp, pose)
            self.writer.writerow((stamp_ns, pose.x(), pose.y(), pose.theta(),
                                  self.estimator.bias, self.status.get('overlap'),
                                  self.status.get('rmse_m'), elapsed))
            self.csv_stream.flush()
        except (TransformException, ValueError) as exc:
            self.status = {'state': 'waiting_for_lidar_tf_or_valid_scan', 'detail': str(exc)}
        except RuntimeError as exc:
            self.estimator.failed = 'optimizer_error_restart_required'
            self.status = {'state': self.estimator.failed, 'detail': str(exc)}

    def publish_pose(self, stamp, pose):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.settings['output_frame']
        msg.child_frame_id = self.settings['base_frame']
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose.x(), pose.y()
        msg.pose.pose.orientation.z = math.sin(pose.theta() / 2)
        msg.pose.pose.orientation.w = math.cos(pose.theta() / 2)
        # Pose2 marginal covariance is in local tangent coordinates; rotate to output axes.
        c, s = math.cos(pose.theta()), math.sin(pose.theta())
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        covariance = rotation @ self.estimator.graph.covariance() @ rotation.T
        full = np.diag([1e6] * 6)
        full[np.ix_([0, 1, 5], [0, 1, 5])] = covariance
        msg.pose.covariance = full.ravel().tolist()
        # Velocity is not estimated by this pose-only backend.
        msg.twist.covariance = np.diag([1e6] * 6).ravel().tolist()
        self.odom_pub.publish(msg)
        entry = PoseStamped()
        entry.header, entry.pose = msg.header, msg.pose.pose
        self.path.header.stamp = stamp
        self.path.poses.append(entry)
        self.path_pub.publish(self.path)

    def report(self):
        status = dict(self.status)
        now = time.monotonic()
        if self.last_scan_arrival is None or now - self.last_scan_arrival > self.settings['sensor_timeout']:
            status['state'] = 'scan_stream_stale'
        elif self.config.use_imu and (self.last_imu_arrival is None or
                                     now - self.last_imu_arrival > self.settings['sensor_timeout']):
            status['state'] = 'imu_stream_stale'
        status['monotonic_s'] = now
        status['pose_available'] = self.estimator.graph is not None
        status['pose_fresh'] = status['state'] == 'tracking'
        text = json.dumps(status, allow_nan=False)
        self.status_pub.publish(String(data=text))
        self.status_stream.write(text + '\n')
        self.status_stream.flush()
        self.get_logger().info(text)

    def destroy_node(self):
        self.csv_stream.close()
        self.status_stream.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = JetAutoGtsam()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
