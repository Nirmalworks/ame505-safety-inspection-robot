#!/usr/bin/env python3
"""Astra depth image -> /depth_cam/points (PointCloud2) for the Nav2 costmaps + collision monitor.

depth_image_proc is not installed on this robot, so this does the same in numpy: decimated
(every 4th pixel -> 80x60), 0.2-3.0 m, in the camera's optical frame (depth_cam_frame).

The ROS 1 -> ROS 2 camera bridge stamps frames on arrival although they are ~1-2 s old, so
while the robot ROTATES fast, depth points would land in the wrong place. Frames are skipped
while |yaw rate| > max_yaw_rate (from /odom); the lidar keeps covering those moments.

    ros2 run ame505_bringup depth_obstacles
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DepthObstacles(Node):
    def __init__(self):
        super().__init__('depth_obstacles')
        self.step = int(self.declare_parameter('step', 4).value)
        self.min_z = float(self.declare_parameter('min_range', 0.2).value)
        self.max_z = float(self.declare_parameter('max_range', 3.0).value)
        self.max_yaw_rate = float(self.declare_parameter('max_yaw_rate', 0.35).value)
        self.K = None
        self.rays = None
        self.yaw_rate = 0.0
        self.pub = self.create_publisher(PointCloud2, '/depth_cam/points', qos_profile_sensor_data)
        # depth is registered to the colour image -> colour intrinsics apply
        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.on_info, 5)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                       for i, n in enumerate('xyz')]

    def on_odom(self, m):
        self.yaw_rate = m.twist.twist.angular.z

    def on_info(self, m):
        if self.K is None and m.k[0] > 0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])
            self.get_logger().info(f'camera intrinsics fx={m.k[0]:.1f} fy={m.k[4]:.1f}')

    def on_depth(self, m):
        if self.K is None or m.encoding not in ('16UC1', 'mono16'):
            return
        if abs(self.yaw_rate) > self.max_yaw_rate:
            return
        d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
        s = self.step
        z = d[::s, ::s].astype(np.float32) / 1000.0
        if self.rays is None or self.rays[0].shape != z.shape:
            fx, fy, cx, cy = self.K
            v, u = np.mgrid[0:m.height:s, 0:m.width:s]
            self.rays = ((u - cx) / fx, (v - cy) / fy)
        ok = (z > self.min_z) & (z < self.max_z)
        pts = np.stack([self.rays[0][ok] * z[ok], self.rays[1][ok] * z[ok], z[ok]], 1).astype(np.float32)
        out = PointCloud2()
        out.header = m.header
        out.height, out.width = 1, len(pts)
        out.fields, out.is_bigendian, out.point_step = self.fields, False, 12
        out.row_step, out.is_dense = 12 * len(pts), True
        out.data = pts.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
