#!/usr/bin/env python3
"""Exercise the actual ROS node with synthetic sensors, TF, and known motion.

Run in an isolated ROS domain: ROS_DOMAIN_ID=77 .venv-ros/bin/python ros_smoke_test.py
No robot hardware or motion commands are used. Requires ROS 2 Humble.
"""

import math
import tempfile
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from live_node import JetAutoGtsam
from synthetic_sensors import base_pose, room_scan


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
        transforms = []
        for name, x, yaw in (('test_lidar', 0.096, math.pi), ('test_imu', 0., 0.)):
            tr = TransformStamped()
            tr.header.stamp = self.get_clock().now().to_msg()
            tr.header.frame_id, tr.child_frame_id = 'test_base', name
            tr.transform.translation.x = x
            tr.transform.rotation.z = math.sin(yaw / 2)
            tr.transform.rotation.w = math.cos(yaw / 2)
            transforms.append(tr)
        self.static_tf.sendTransform(transforms)
        self.create_timer(0.02, self.publish_sensors)

    def publish_sensors(self):
        elapsed = time.monotonic() - self.start
        stamp = self.get_clock().now().to_msg()
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


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    with tempfile.TemporaryDirectory() as directory:
        options = {'scan_topic': '/gtsam_test/scan', 'imu_topic': '/gtsam_test/imu',
                   'base_frame': 'test_base', 'output_frame': 'gtsam_test_odom',
                   'calibration_seconds': 1., 'output_dir': directory}
        estimator = JetAutoGtsam(parameter_overrides=[Parameter(k, value=v) for k, v in options.items()])
        sensors = TestSensors()
        executor.add_node(estimator)
        executor.add_node(sensors)
        try:
            while time.monotonic() - sensors.start < 8.:
                executor.spin_once(timeout_sec=0.02)
            if len(sensors.received) < 10:
                raise AssertionError(f'Only {len(sensors.received)} poses; status={estimator.status}')
            last = sensors.received[-1]
            # Reconstruct truth at the published measurement time, not wall-clock now.
            stamp = last.header.stamp.sec + last.header.stamp.nanosec / 1e9
            current = sensors.get_clock().now().nanoseconds / 1e9
            elapsed = time.monotonic() - sensors.start - (current - stamp)
            x, y, yaw = base_pose(elapsed)
            p, q = last.pose.pose.position, last.pose.pose.orientation
            error = math.hypot(p.x - x, p.y - y)
            yaw_error = abs(2 * math.atan2(q.z, q.w) - yaw)
            if error > 0.05 or yaw_error > 0.04:
                raise AssertionError(f'Position error={error:.3f} m; yaw error={yaw_error:.3f} rad')
            print(f'PASS: {len(sensors.received)} poses; error={error:.3f} m, yaw={yaw_error:.3f} rad')
        finally:
            executor.remove_node(estimator)
            executor.remove_node(sensors)
            estimator.destroy_node()
            sensors.destroy_node()
            executor.shutdown()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
