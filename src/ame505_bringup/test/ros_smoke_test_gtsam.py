#!/usr/bin/env python3
"""Run the real gtsam_slam node on synthetic lidar/IMU/TF with known motion (no robot needed).

Also checks the map anchoring: a fake `test_map -> test_base` TF is published, and the
GTSAM pose expressed in test_map via the node's static TF must match it.
Adapted from Shreyash's ros_smoke_test.py. Run in an isolated ROS domain:

    ROS_DOMAIN_ID=77 python3 src/ame505_bringup/test/ros_smoke_test_gtsam.py
"""
import math
import os
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

sys.path.insert(0, os.path.dirname(__file__))
from synthetic_sensors import base_pose, room_scan  # noqa: E402
from ame505_bringup.gtsam_slam import GtsamSlam      # noqa: E402

MAP_OFFSET = (1.5, -0.7, 0.6)   # where the robot starts in the fake map frame


class TestSensors(Node):
    def __init__(self):
        super().__init__('gtsam_synthetic_sensors')
        self.start = time.monotonic()
        self.counter = 0
        self.received = []
        self.imu = self.create_publisher(Imu, '/gtsam_test/imu', qos_profile_sensor_data)
        self.scan = self.create_publisher(LaserScan, '/gtsam_test/scan', qos_profile_sensor_data)
        self.create_subscription(Odometry, '/gtsam/odom', self.received.append, 10)
        self.static_tf = StaticTransformBroadcaster(self)
        self.tf = TransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        transforms = []
        for name, x, yaw in (('test_lidar', 0.096, math.pi), ('test_imu', 0., -math.pi / 2)):
            tr = TransformStamped()
            tr.header.stamp = self.get_clock().now().to_msg()
            tr.header.frame_id, tr.child_frame_id = 'test_base', name
            tr.transform.translation.x = x
            tr.transform.rotation.z = math.sin(yaw / 2)
            tr.transform.rotation.w = math.cos(yaw / 2)
            transforms.append(tr)
        self.static_tf.sendTransform(transforms)
        self.create_timer(0.02, self.publish_sensors)

    def truth_in_map(self, elapsed):
        x, y, yaw = base_pose(elapsed)
        ox, oy, oyaw = MAP_OFFSET
        c, s = math.cos(oyaw), math.sin(oyaw)
        return ox + c * x - s * y, oy + s * x + c * y, oyaw + yaw

    def publish_sensors(self):
        elapsed = time.monotonic() - self.start
        stamp = self.get_clock().now().to_msg()
        mx, my, myaw = self.truth_in_map(elapsed)       # "RTAB-Map" pose of the robot
        tr = TransformStamped()
        tr.header.stamp, tr.header.frame_id, tr.child_frame_id = stamp, 'test_map', 'test_base'
        tr.transform.translation.x, tr.transform.translation.y = mx, my
        tr.transform.rotation.z, tr.transform.rotation.w = math.sin(myaw / 2), math.cos(myaw / 2)
        self.tf.sendTransform(tr)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'test_imu'
        imu.angular_velocity.z = (0.04 if elapsed > 3. else 0.) + 0.001
        self.imu.publish(imu)
        self.counter += 1
        if self.counter % 5:
            return
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'test_lidar'
        scan.angle_min, scan.angle_max = -math.pi / 2, math.pi / 2
        scan.angle_increment = math.pi / 359
        scan.range_min, scan.range_max, scan.scan_time = 0.1, 10., 0.1
        scan.ranges = room_scan(base_pose(elapsed)).tolist()
        self.scan.publish(scan)


def yaw_of(q):
    return 2 * math.atan2(q.z, q.w)


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    with tempfile.TemporaryDirectory() as directory:
        options = {'scan_topic': '/gtsam_test/scan', 'imu_topic': '/gtsam_test/imu',
                   'base_frame': 'test_base', 'output_frame': 'gtsam_test_odom',
                   'anchor_frames': ['test_map'], 'calibration_seconds': 1., 'output_dir': directory}
        estimator = GtsamSlam(parameter_overrides=[Parameter(k, value=v) for k, v in options.items()])
        sensors = TestSensors()
        executor.add_node(estimator)
        executor.add_node(sensors)
        try:
            while time.monotonic() - sensors.start < 8.:
                executor.spin_once(timeout_sec=0.02)
            if len(sensors.received) < 10:
                raise AssertionError(f'Only {len(sensors.received)} poses; status={estimator.status}')
            last = sensors.received[-1]
            stamp = last.header.stamp.sec + last.header.stamp.nanosec / 1e9
            current = sensors.get_clock().now().nanoseconds / 1e9
            elapsed = time.monotonic() - sensors.start - (current - stamp)
            x, y, yaw = base_pose(elapsed)
            p, q = last.pose.pose.position, last.pose.pose.orientation
            error = math.hypot(p.x - x, p.y - y)
            yaw_error = abs(yaw_of(q) - yaw)
            if error > 0.05 or yaw_error > 0.04:
                raise AssertionError(f'Position error={error:.3f} m; yaw error={yaw_error:.3f} rad')
            # Anchoring: GTSAM pose expressed in test_map must land on the robot's map pose.
            if not estimator.anchor or estimator.anchor['frame'] != 'test_map':
                raise AssertionError(f'Not anchored: {estimator.anchor}')
            a = sensors.buffer.lookup_transform('test_map', 'gtsam_test_odom', Time()).transform
            ayaw = yaw_of(a.rotation)
            mx = a.translation.x + math.cos(ayaw) * p.x - math.sin(ayaw) * p.y
            my = a.translation.y + math.sin(ayaw) * p.x + math.cos(ayaw) * p.y
            tx, ty, _ = sensors.truth_in_map(elapsed)
            map_error = math.hypot(mx - tx, my - ty)
            if map_error > 0.06:
                raise AssertionError(f'GTSAM pose in map frame off by {map_error:.3f} m')
            # Re-align service
            client = sensors.create_client(Trigger, '/gtsam/reanchor')
            fut = client.call_async(Trigger.Request())
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 3:
                executor.spin_once(timeout_sec=0.02)
            if not (fut.done() and fut.result().success):
                raise AssertionError('reanchor service failed')
            print(f'PASS: {len(sensors.received)} poses; odom error={error:.3f} m, yaw={yaw_error:.3f} rad; '
                  f'on map error={map_error:.3f} m; reanchor: {fut.result().message}')
        finally:
            executor.remove_node(estimator)
            executor.remove_node(sensors)
            estimator.destroy_node()
            sensors.destroy_node()
            executor.shutdown()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
