#!/usr/bin/env python3
"""Simulated JetAuto base + lidar, so mapping / AMCL / Nav2 / patrol / dashboard run on a laptop.

Stands in for the Hiwonder drivers, the RPLidar + laser filter and the EKF (robot.launch.py
starts this instead of the hardware when AME505_SIM=1). No Gazebo: the robot is a point that
follows velocity commands, and the lidar is ray-cast against a floor-plan image.

  in : /cmd_vel (Nav2), /controller/cmd_vel (teleop, E-STOP)      geometry_msgs/Twist
  out: /scan  360 beams in lidar_frame, only the robot-front 180 deg are valid (like the A1 filter)
       /odom + TF odom -> base_footprint  (drifts: the wheels slip, as on the real robot)
       /imu, /ros_robot_controller/battery, /sim/ground_truth (true pose in the map frame)

Not simulated: the depth camera, wheel dynamics, the motor board keeping its last speed.
"""
import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import UInt16
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

BEAMS = 360
STEP_HZ = 30.0


def quat_z(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class SimRobot(Node):
    def __init__(self):
        super().__init__('sim_robot')
        p = self.declare_parameter
        world_yaml = p('world', '').value
        self.pose = np.array([p('start_x', 0.0).value, p('start_y', 0.0).value, p('start_yaw', 0.0).value])
        self.max_v = np.array([p('max_vx', 0.25).value, p('max_vy', 0.2).value, p('max_wz', 1.0).value])
        # true motion = command * slip; odometry believes the command (wheel encoders)
        self.slip = np.array([p('slip_x', 0.97).value, p('slip_y', 0.88).value, p('slip_yaw', 0.95).value])
        self.radius = p('robot_radius', 0.18).value          # collision circle (footprint 0.33 x 0.28 m)
        self.range_max = p('range_max', 12.0).value
        self.range_min = p('range_min', 0.2).value           # the A1 filter drops < 0.2 m
        self.range_noise = p('range_noise', 0.01).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value

        meta = yaml.safe_load(open(world_yaml))
        img = cv2.imread(os.path.join(os.path.dirname(world_yaml), meta['image']), cv2.IMREAD_UNCHANGED)
        self.res, self.origin = meta['resolution'], meta['origin'][:2]
        self.occ = img < 100                                  # walls; unknown counts as empty space
        # distance (m) from every cell to the nearest wall, for the collision check
        self.clearance = cv2.distanceTransform((~self.occ).astype(np.uint8), cv2.DIST_L2, 5) * self.res

        # Optional moving people (sim/sim_hall_people.yaml): circles the lidar sees and the robot
        # cannot drive through. walker: paces between a and b; stander: stands at a spot part of the time.
        people_yaml = p('people', '').value                   # '' = nobody, 'default' = <world>_people.yaml
        if people_yaml == 'default':
            people_yaml = world_yaml.replace('_world.yaml', '_people.yaml')
        self.people = (yaml.safe_load(open(people_yaml)) or {}).get('people', []) if people_yaml else []
        for q in self.people:
            q.setdefault('radius', 0.2)
            q['pos'] = np.array(q['a'] if q['type'] == 'walker' else q['at'], float)
            q.update(to_b=True, wait=0.0, held=0.0, here=q['type'] == 'walker')
        self.contacts, self.in_contact = 0, False
        self.t0 = None

        self.odom = np.zeros(3)                               # pose in the odom frame (starts at 0)
        self.cmd, self.cmd_stamp = np.zeros(3), 0.0
        self.vel = np.zeros(3)
        self.lidar = None                                     # (x, y, yaw) of lidar_frame in base_footprint
        self.bumps = 0
        self.rng = np.random.default_rng(505)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Twist, '/controller/cmd_vel', self.on_cmd, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', qos_profile_sensor_data)
        self.truth_pub = self.create_publisher(PoseStamped, '/sim/ground_truth', 10)
        self.batt_pub = self.create_publisher(UInt16, '/ros_robot_controller/battery', 10)
        self.create_timer(1.0 / STEP_HZ, self.step)
        self.create_timer(1.0 / 12.0, self.scan)              # RPLidar A1: 12 Hz
        self.create_timer(1.0, lambda: self.batt_pub.publish(UInt16(data=12000)))
        if self.people:
            self.people_pub = self.create_publisher(MarkerArray, '/sim/people', 5)
            self.create_timer(0.2, self.publish_people)
        self.get_logger().info(f'SIMULATED robot in {world_yaml}, start {self.pose.tolist()}, '
                               f'{len(self.people)} simulated people')

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_cmd(self, msg):
        self.cmd = np.clip([msg.linear.x, msg.linear.y, msg.angular.z], -self.max_v, self.max_v)
        self.cmd_stamp = self.now_s()

    def blocked(self, x, y):
        c = int((x - self.origin[0]) / self.res)
        r = self.occ.shape[0] - 1 - int((y - self.origin[1]) / self.res)
        if not (0 <= r < self.occ.shape[0] and 0 <= c < self.occ.shape[1]):
            return True
        return self.clearance[r, c] < self.radius

    def move_people(self, dt):
        if self.t0 is None:
            self.t0 = self.now_s()
        t = self.now_s() - self.t0
        robot = self.pose[:2]
        for q in self.people:
            near = np.linalg.norm(q['pos'] - robot) < q['radius'] + self.radius + 0.35
            if q['type'] == 'stander':                        # e.g. someone chatting in a doorway
                want = (t + q.get('phase', 0.0)) % q['period'] < q['duration']
                q['here'] = want and (q['here'] or not near)  # never appears on top of the robot
                continue
            if q['wait'] > 0:
                q['wait'] -= dt
                continue
            goal = np.array(q['b'] if q['to_b'] else q['a'], float)
            d = goal - q['pos']
            dist = float(np.linalg.norm(d))
            if dist < 0.05:                                   # arrived: pause, then walk back
                q['to_b'], q['wait'] = not q['to_b'], q.get('pause', 3.0)
                continue
            nxt = q['pos'] + d / dist * min(q.get('speed', 0.7) * dt, dist)
            if np.linalg.norm(nxt - robot) < q['radius'] + self.radius + 0.35 and q['held'] < 3.0:
                q['held'] += dt                               # the robot is in the way: hesitate ~3 s,
                continue
            touch = q['radius'] + self.radius + 0.03          # ...then step around it, like people do
            off = nxt - robot
            if np.linalg.norm(off) < touch:
                nxt = robot + off / max(float(np.linalg.norm(off)), 1e-6) * touch
            q['pos'] = nxt
            if np.linalg.norm(nxt - robot) > q['radius'] + self.radius + 0.6:
                q['held'] = 0.0                               # clear of the robot again

    def touches_person(self, x, y):
        return any(q['here'] and math.hypot(q['pos'][0] - x, q['pos'][1] - y) < q['radius'] + self.radius
                   for q in self.people)

    def publish_people(self):
        arr = MarkerArray()
        for i, q in enumerate(self.people):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', self.get_clock().now().to_msg()
            m.ns, m.id, m.type = 'people', i, Marker.CYLINDER
            m.action = Marker.ADD if q['here'] else Marker.DELETE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(q['pos'][0]), float(q['pos'][1]), 0.85
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2 * q['radius']
            m.scale.z = 1.7
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.1, 0.9
            arr.markers.append(m)
        self.people_pub.publish(arr)

    def step(self):
        dt = 1.0 / STEP_HZ
        self.move_people(dt)
        target = self.cmd if self.now_s() - self.cmd_stamp < self.cmd_timeout else np.zeros(3)
        self.vel += np.clip(target - self.vel, -1.5 * dt, 1.5 * dt)      # 1.5 m/s^2 (rad/s^2) ramp
        true_v = self.vel * self.slip
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        nx = self.pose[0] + (true_v[0] * c - true_v[1] * s) * dt
        ny = self.pose[1] + (true_v[0] * s + true_v[1] * c) * dt
        hit_person = self.touches_person(nx, ny)
        if hit_person and not self.in_contact and float(np.hypot(*true_v[:2])) > 0.02:
            self.contacts += 1                                # the robot drove into someone
            self.get_logger().error(f'CONTACT: the robot touched a person at ({nx:.2f}, {ny:.2f}) '
                                    f'while moving at {float(np.hypot(*true_v[:2])):.2f} m/s')
        self.in_contact = hit_person
        if hit_person:
            pass                                              # cannot push people: stay put
        elif self.blocked(nx, ny):                            # walls stop the robot; it can still turn
            self.bumps += 1
            if self.bumps % 30 == 1:
                self.get_logger().warn(f'BUMP: the robot is pushing against a wall at '
                                       f'({self.pose[0]:.2f}, {self.pose[1]:.2f})')
        else:
            self.pose[0], self.pose[1] = nx, ny
        self.pose[2] += true_v[2] * dt
        # odometry: wheel encoders report the commanded motion; yaw comes from the gyro (true rate)
        co, so = math.cos(self.odom[2]), math.sin(self.odom[2])
        self.odom[0] += (self.vel[0] * co - self.vel[1] * so) * dt
        self.odom[1] += (self.vel[0] * so + self.vel[1] * co) * dt
        self.odom[2] += (true_v[2] + 0.002) * dt                         # small gyro bias
        self.publish_state(true_v)

    def publish_state(self, true_v):
        stamp = self.get_clock().now().to_msg()
        qz, qw = quat_z(self.odom[2])
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = float(self.odom[0]), float(self.odom[1])
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf_pub.sendTransform(tf)

        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, 'odom', 'base_footprint'
        od.pose.pose.position.x, od.pose.pose.position.y = float(self.odom[0]), float(self.odom[1])
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.linear.y = float(self.vel[0]), float(self.vel[1])
        od.twist.twist.angular.z = float(true_v[2])
        self.odom_pub.publish(od)

        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'imu_link'
        imu.orientation.z, imu.orientation.w = qz, qw
        imu.angular_velocity.z = float(true_v[2])
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)

        gt = PoseStamped()
        gt.header.stamp, gt.header.frame_id = stamp, 'map'
        gt.pose.position.x, gt.pose.position.y = float(self.pose[0]), float(self.pose[1])
        gt.pose.orientation.z, gt.pose.orientation.w = quat_z(self.pose[2])
        self.truth_pub.publish(gt)

    def lidar_mount(self):
        if self.lidar is None:
            try:
                t = self.tf_buffer.lookup_transform('base_footprint', 'lidar_frame', Time()).transform
                yaw = 2.0 * math.atan2(t.rotation.z, t.rotation.w)
                self.lidar = (t.translation.x, t.translation.y, yaw)
            except Exception:  # noqa: BLE001  (robot_state_publisher not up yet)
                return None
        return self.lidar

    def scan(self):
        mount = self.lidar_mount()
        if mount is None:
            return
        lx, ly, lyaw = mount
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        ox = self.pose[0] + lx * c - ly * s
        oy = self.pose[1] + lx * s + ly * c
        inc = 2.0 * math.pi / BEAMS
        angles = -math.pi + inc * np.arange(BEAMS)            # in lidar_frame
        in_base = np.arctan2(np.sin(angles + lyaw), np.cos(angles + lyaw))
        front = np.abs(in_base) <= math.pi / 2                # the A1 filter keeps the robot-front half
        world = angles[front] + lyaw + self.pose[2]
        dist = np.arange(self.range_min, self.range_max, self.res / 2)
        cols = ((ox + np.outer(np.cos(world), dist) - self.origin[0]) / self.res).astype(int)
        rows = self.occ.shape[0] - 1 - ((oy + np.outer(np.sin(world), dist) - self.origin[1]) / self.res).astype(int)
        inside = (rows >= 0) & (rows < self.occ.shape[0]) & (cols >= 0) & (cols < self.occ.shape[1])
        hit = np.zeros(inside.shape, bool)
        hit[inside] = self.occ[rows[inside], cols[inside]]
        first = hit.argmax(axis=1)
        r = np.where(hit.any(axis=1), dist[first] + self.rng.normal(0, self.range_noise, len(first)), np.inf)
        for q in self.people:                                 # people: ray against a circle
            if not q['here']:
                continue
            ocx, ocy = q['pos'][0] - ox, q['pos'][1] - oy
            along = ocx * np.cos(world) + ocy * np.sin(world)
            disc = along ** 2 - (ocx ** 2 + ocy ** 2 - q['radius'] ** 2)
            t = along - np.sqrt(np.maximum(disc, 0.0))
            seen = (disc >= 0) & (t > self.range_min) & (t < self.range_max)
            r = np.where(seen & (t < r), t + self.rng.normal(0, self.range_noise, len(t)), r)

        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'lidar_frame'
        msg.angle_min, msg.angle_max, msg.angle_increment = -math.pi, math.pi - inc, inc
        msg.scan_time, msg.time_increment = 1.0 / 12.0, 0.0
        msg.range_min, msg.range_max = self.range_min, self.range_max
        ranges = np.full(BEAMS, np.inf)
        ranges[front] = r
        msg.ranges = ranges.astype(np.float32).tolist()
        self.scan_pub.publish(msg)


def main():
    rclpy.init()
    node = SimRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
