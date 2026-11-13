#!/usr/bin/env python3
"""Live GTSAM lidar/gyro odometry (Shreyash's live_node.py, integrated into ame505_bringup).

    /scan + sensor TF -> point-to-line ICP ---\
    /imu  + sensor TF -> calibrated gyro yaw --+-> GTSAM iSAM2 -> /gtsam/odom /gtsam/path /gtsam/status

Runs NEXT TO the normal robot software (drive / mapping / navigation) - read-only: it never
publishes drive commands and never touches map/odom TF. Its own frame `gtsam_odom` starts
where the robot is when tracking begins; a static TF `map -> gtsam_odom` (or `odom ->
gtsam_odom` when no map is running) is published once so RViz and the dashboard can draw the
GTSAM path ON the existing map. Call /gtsam/reanchor (dashboard: "Re-align to map") to
re-attach it after RTAB-Map has localized the robot.

Keep the robot STILL for ~4 s after start (gyro bias calibration).

    ros2 launch ame505_bringup gtsam_slam.launch.py              # lidar + gyro
    ros2 launch ame505_bringup gtsam_slam.launch.py use_imu:=false   # lidar only
"""
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from ame505_bringup.paths import data_dir

try:
    import gtsam
    from ame505_bringup.gtsam_core import Config, HardwareOdometry, rotation_matrix, scan_points
except ImportError as _exc:  # pragma: no cover - only when GTSAM is not installed
    raise SystemExit(f'GTSAM is not installed ({_exc}). Run: scripts/setup_gtsam.sh')


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GtsamSlam(Node):
    def __init__(self, **kwargs):
        super().__init__('gtsam_slam', **kwargs)
        defaults = {
            **asdict(Config()), 'scan_topic': '/scan', 'imu_topic': '/imu',
            'base_frame': 'base_footprint', 'output_frame': 'gtsam_odom',
            'anchor_frames': ['map', 'odom'],
            'process_rate_hz': 5.0, 'sensor_timeout': 1.0, 'output_dir': '',
        }
        self.declare_parameters('', list(defaults.items()))
        params = {name: self.get_parameter(name).value for name in defaults}
        params['anchor_frames'] = [f for f in params['anchor_frames'] if f]
        self.config = Config(**{name: params[name] for name in asdict(Config())})
        if params['process_rate_hz'] <= 0 or params['sensor_timeout'] <= 0:
            raise ValueError('process_rate_hz and sensor_timeout must be positive')
        if params['output_frame'] in ('map', 'odom', params['base_frame']):
            raise ValueError('Use a separate output frame, such as gtsam_odom')
        self.settings = params
        self.estimator = HardwareOdometry(self.config)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.anchor = None           # {'frame', 'x', 'y', 'yaw'}: pose of gtsam_odom in that frame
        self.last_pose = None        # (stamp msg, gtsam.Pose2)
        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        imu_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, params['scan_topic'], self.on_scan, scan_qos)
        if self.config.use_imu:
            self.create_subscription(Imu, params['imu_topic'], self.on_imu, imu_qos)
        self.odom_pub = self.create_publisher(Odometry, '/gtsam/odom', 10)
        self.path_pub = self.create_publisher(PathMsg, '/gtsam/path', 1)
        self.status_pub = self.create_publisher(String, '/gtsam/status', 10)
        self.create_service(Trigger, '/gtsam/reanchor', self.on_reanchor)
        self.path = PathMsg()
        self.path.header.frame_id = params['output_frame']
        self.pending = None
        self.last_scan_arrival = None
        self.last_imu_arrival = None
        self.imu_tf_error = None
        self.status = {'state': 'waiting_for_scan'}
        root = Path(params['output_dir']).expanduser() if params['output_dir'] else Path(data_dir('gtsam'))
        output = root / (time.strftime('%Y%m%d_%H%M%S') + '_live')
        output.mkdir(parents=True, exist_ok=True)
        (output / 'parameters.json').write_text(json.dumps(params, indent=2) + '\n')
        self.csv_stream = (output / 'trajectory.csv').open('w', newline='')
        self.writer = csv.writer(self.csv_stream)
        self.writer.writerow(('stamp_ns', 'x', 'y', 'yaw', 'gyro_bias_rad_s',
                              'overlap', 'rmse_m', 'processing_ms'))
        self.status_stream = (output / 'status.jsonl').open('w')
        self.create_timer(1 / params['process_rate_hz'], self.process)
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            f'Reading {params["scan_topic"]}' + (f', {params["imu_topic"]}' if self.config.use_imu else ' (lidar only)')
            + f'; output frame {params["output_frame"]}; logs: {output}. '
            'Keep robot STILL for the initial gyro calibration.')

    # ---------------- sensors (unchanged from Shreyash's live_node.py) ----------------
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
            self.estimator.gyro.add(Time.from_msg(msg.header.stamp).nanoseconds / 1e9, float(rate))
            self.last_imu_arrival = time.monotonic()
            self.imu_tf_error = None
        except (TransformException, ValueError) as exc:
            self.imu_tf_error = str(exc)

    def on_scan(self, msg):
        self.pending = msg  # keep the newest scan instead of accumulating latency
        self.last_scan_arrival = time.monotonic()

    def process(self):
        if self.pending is None:
            return
        msg = self.pending
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        stamp = stamp_ns / 1e9
        # Give the ~50 Hz IMU a short chance to bracket the scan timestamp.
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
                                 msg.range_min, msg.range_max, translation, quaternion, self.config)
            pose, self.status = self.estimator.process(stamp, points)
            if pose is None:
                return
            elapsed = (time.monotonic() - started) * 1000
            self.status['processing_ms'] = elapsed
            self.last_pose = (msg.header.stamp, pose)
            self.update_anchor()
            self.publish_pose(msg.header.stamp, pose)
            self.writer.writerow((stamp_ns, pose.x(), pose.y(), pose.theta(), self.estimator.bias,
                                  self.status.get('overlap'), self.status.get('rmse_m'), elapsed))
            self.csv_stream.flush()
        except (TransformException, ValueError) as exc:
            self.status = {'state': 'waiting_for_lidar_tf_or_valid_scan', 'detail': str(exc)}
        except RuntimeError as exc:
            self.estimator.failed = 'optimizer_error_restart_required'
            self.status = {'state': self.estimator.failed, 'detail': str(exc)}

    # ---------------- anchoring to the existing map ----------------
    def _robot_in(self, frame, stamp):
        for t in (Time.from_msg(stamp), Time()):          # at the scan time, else latest
            try:
                tr = self.tf_buffer.lookup_transform(frame, self.settings['base_frame'], t)
                return gtsam.Pose2(tr.transform.translation.x, tr.transform.translation.y,
                                   _yaw(tr.transform.rotation))
            except TransformException:
                continue
        return None

    def anchor_to(self, frame):
        """gtsam_odom pose in `frame` such that the current GTSAM pose lands on the robot's pose there."""
        if self.last_pose is None:
            return False, 'No GTSAM pose yet (still calibrating?)'
        stamp, pose = self.last_pose
        robot = self._robot_in(frame, stamp)
        if robot is None:
            return False, f'No TF {frame} -> {self.settings["base_frame"]} yet'
        a = robot.compose(pose.inverse())
        tr = TransformStamped()
        tr.header.stamp = self.get_clock().now().to_msg()
        tr.header.frame_id, tr.child_frame_id = frame, self.settings['output_frame']
        tr.transform.translation.x, tr.transform.translation.y = a.x(), a.y()
        tr.transform.rotation.z, tr.transform.rotation.w = math.sin(a.theta() / 2), math.cos(a.theta() / 2)
        self.static_tf.sendTransform(tr)
        self.anchor = {'frame': frame, 'x': round(a.x(), 3), 'y': round(a.y(), 3), 'yaw': round(a.theta(), 4)}
        msg = f'GTSAM path anchored in "{frame}" at ({a.x():.2f}, {a.y():.2f}, {math.degrees(a.theta()):.0f} deg)'
        self.get_logger().info(msg)
        return True, msg

    def update_anchor(self):
        """Anchor once; upgrade odom -> map automatically when a map frame appears later."""
        frames = self.settings['anchor_frames']
        best = frames.index(self.anchor['frame']) if self.anchor and self.anchor['frame'] in frames else len(frames)
        for frame in frames[:best]:
            if self.anchor_to(frame)[0]:
                return

    def on_reanchor(self, _req, res):
        res.success, res.message = False, 'No anchor frame available'
        for frame in self.settings['anchor_frames']:
            res.success, res.message = self.anchor_to(frame)
            if res.success:
                break
        return res

    # ---------------- outputs ----------------
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
        msg.twist.covariance = np.diag([1e6] * 6).ravel().tolist()   # velocity not estimated
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
        status['use_imu'] = self.config.use_imu
        status['anchor'] = self.anchor
        if self.last_pose is not None:
            p = self.last_pose[1]
            status['pose'] = [round(p.x(), 3), round(p.y(), 3), round(p.theta(), 4)]
        text = json.dumps(status, allow_nan=False, default=float)
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
        node = GtsamSlam()
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
