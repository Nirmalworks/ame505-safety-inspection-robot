#!/usr/bin/env python3
"""AME 505 robot dashboard: live sensors + camera, one-click mapping / save / 3D viewer.

    ros2 run ame505_bringup dashboard            # then open http://localhost:8505
    (from a laptop on the same Wi-Fi: http://<robot-ip>:8505)

The dashboard starts and stops the ROS launch files itself, so you only need this one
command. Default pipeline is 2D: slam_toolbox mapping (maps_2d/<map>.pgm/.yaml) and
map_server + AMCL lidar localization for Nav2. Legacy RTAB-Map 3D maps (maps/<map>.db)
still work when no 2D map of that name exists. It listens to topics read-only, except E-STOP (zero velocity) and saving maps.
"""
import collections
import glob
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from flask import Flask, Response, abort, jsonify, request, send_from_directory
import yaml
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, Joy, LaserScan, PointCloud2
from std_msgs.msg import String, UInt16
from std_srvs.srv import Empty, Trigger

from ame505_bringup.motor_stop import board_stop
from ame505_bringup.paths import checkpoints_file, data_dir, map_db, map_yaml
from ame505_bringup.reconstruct_3d import mat_uncompress
from ame505_bringup.save_3d_map import write_ply

PORT = int(os.environ.get('AME505_DASHBOARD_PORT', '8505'))
WEB = os.path.join(get_package_share_directory('ame505_bringup'), 'web')
LIDAR_YAW = math.pi   # lidar_frame_joint rpy="0 0 pi" in ame505_description/urdf/lidar_a1.urdf.xacro
NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')
BUTTONS = ['A', 'B', 'X', 'Y', 'Back', 'Xbox', 'Start', 'LS', 'RS', 'LB', 'RB',
           'Up', 'Down', 'Left', 'Right']


class Rate:
    def __init__(self):
        self.t = collections.deque(maxlen=60)

    def tick(self):
        self.t.append(time.time())

    def hz(self):
        now = time.time()
        recent = [x for x in self.t if now - x < 3.0]
        if len(recent) < 2:
            return 0.0
        return round((len(recent) - 1) / max(recent[-1] - recent[0], 1e-3), 1)


class Sensors(Node):
    def __init__(self):
        super().__init__('ame505_dashboard')
        self.lock = threading.Lock()
        self.rate = collections.defaultdict(Rate)
        self.v = {}
        self.scan_pts = []
        self.jpeg = {'rgb': None, 'depth': None}
        self.frame_id = {'rgb': 0, 'depth': 0}
        sd = qos_profile_sensor_data
        self.create_subscription(LaserScan, '/scan', self.on_scan, sd)
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.on_rgb, sd)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self.on_depth, sd)
        self.create_subscription(Odometry, '/odom', self.on_odom, sd)
        self.create_subscription(Imu, '/imu', self.on_imu, sd)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_subscription(UInt16, '/ros_robot_controller/battery', self.on_batt, 10)
        self.create_subscription(Twist, '/controller/cmd_vel', lambda m: self.on_cmd('teleop', m), 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.on_cmd('nav2', m), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        # AMCL (2D navigation): pose + uncertainty, and the particle cloud for the live map
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        self.create_subscription(ParticleCloud, '/particle_cloud', self.on_particles, sd)
        self.amcl_nomotion = self.create_client(Empty, '/request_nomotion_update')
        self.amcl, self.particles = None, []
        # GTSAM lidar/gyro odometry (gtsam_slam node)
        self.create_subscription(String, '/gtsam/status', self.on_gtsam_status, 10)
        self.create_subscription(Path, '/gtsam/path', self.on_gtsam_path, 1)
        self.gtsam_reanchor = self.create_client(Trigger, '/gtsam/reanchor')
        self.gtsam_path, self.gtsam_frame = [], 'gtsam_odom'
        self.estop_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_cancel_all = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.goal_handle = None
        self.goal = None
        self.nav_state = {'state': 'idle'}
        self.map_png, self.map_info, self.map_ver = None, None, 0
        self.plan = []
        self.last_scan = None

    def set(self, key, val, topic):
        with self.lock:
            self.v[key] = val
        self.rate[topic].tick()

    def on_scan(self, m):
        self.last_scan = m
        r = np.asarray(m.ranges, dtype=np.float32)
        ok = np.isfinite(r) & (r > max(m.range_min, 0.12)) & (r < m.range_max)
        # lidar_frame is mounted yawed by pi in the URDF -> rotate so robot-forward is up
        ang = m.angle_min + np.arange(len(r)) * m.angle_increment + LIDAR_YAW
        pts = np.stack([ang[ok], r[ok]], 1)[::3]
        with self.lock:
            self.scan_pts = np.round(pts, 3).tolist()
        self.set('scan', {'rays': len(r), 'valid': int(ok.sum()),
                          'nearest': round(float(r[ok].min()), 2) if ok.any() else None}, '/scan')

    def _img(self, m):
        buf = np.frombuffer(m.data, dtype=np.uint8)
        if m.encoding in ('bgr8', 'rgb8'):
            img = buf.reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
            return img if m.encoding == 'bgr8' else img[:, :, ::-1]
        if m.encoding in ('16UC1', 'mono16'):
            return buf.view(np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
        return None

    def on_rgb(self, m):
        img = self._img(m)
        if img is not None:
            ok, j = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                with self.lock:
                    self.jpeg['rgb'] = j.tobytes()
                    self.frame_id['rgb'] += 1
        self.set('rgb', {'w': m.width, 'h': m.height, 'enc': m.encoding}, '/depth_cam/rgb/image_raw')

    def on_depth(self, m):
        d = self._img(m)
        if d is None:
            return
        valid = d[d > 0]
        center = d[d.shape[0] // 2 - 5:d.shape[0] // 2 + 5, d.shape[1] // 2 - 5:d.shape[1] // 2 + 5]
        cv = center[center > 0]
        vis = np.clip((d.astype(np.float32) - 300) / (4000 - 300), 0, 1)
        vis = cv2.applyColorMap((255 - vis * 255).astype(np.uint8), cv2.COLORMAP_JET)
        vis[d == 0] = 0
        h, w = vis.shape[:2]
        cv2.drawMarker(vis, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 14, 1)
        ok, j = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with self.lock:
                self.jpeg['depth'] = j.tobytes()
                self.frame_id['depth'] += 1
        self.set('depth', {
            'center_m': round(float(np.median(cv)) / 1000, 2) if cv.size else None,
            'valid_pct': round(100.0 * valid.size / d.size, 1),
            'min_m': round(float(valid.min()) / 1000, 2) if valid.size else None,
        }, '/depth_cam/depth/image_raw')

    def on_odom(self, m):
        p, q, t = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.set('odom', {'x': round(p.x, 2), 'y': round(p.y, 2), 'yaw_deg': round(math.degrees(yaw), 1),
                          'vx': round(t.linear.x, 2), 'vy': round(t.linear.y, 2),
                          'wz': round(t.angular.z, 2)}, '/odom')

    def on_imu(self, m):
        a, g = m.linear_acceleration, m.angular_velocity
        self.set('imu', {'ax': round(a.x, 2), 'ay': round(a.y, 2), 'az': round(a.z, 2),
                         'gz': round(g.z, 3)}, '/imu')

    def on_joy(self, m):
        pressed = [BUTTONS[i] if i < len(BUTTONS) else f'b{i}' for i, b in enumerate(m.buttons) if b]
        self.set('joy', {'pressed': pressed, 'axes': [round(a, 2) for a in m.axes[:4]],
                         'lb': len(m.buttons) > 9 and m.buttons[9] == 1,
                         'layout_ok': len(m.axes) == 6}, '/joy')

    def on_batt(self, m):
        self.set('battery', {'v': round(m.data / 1000.0, 2)}, '/ros_robot_controller/battery')

    def on_cmd(self, src, m):
        self.set('cmd_' + src, {'x': round(m.linear.x, 2), 'y': round(m.linear.y, 2),
                                'z': round(m.angular.z, 2)}, '/cmd_' + src)

    def on_map(self, m):
        i = m.info
        known = np.asarray(m.data, dtype=np.int8)
        g = known.reshape(i.height, i.width)
        img = np.full(g.shape, 128, np.uint8)                 # unknown = grey
        img[g == 0] = 235                                     # free = light
        img[g > 0] = np.clip(235 - g[g > 0].astype(np.int32) * 2.35, 0, 235).astype(np.uint8)
        ok, png = cv2.imencode('.png', img[::-1])             # flip: row 0 = top (max y)
        if ok:
            q = i.origin.orientation
            with self.lock:
                self.map_png = png.tobytes()
                self.map_ver += 1
                self.map_info = {'res': i.resolution, 'w': i.width, 'h': i.height,
                                 'ox': i.origin.position.x, 'oy': i.origin.position.y,
                                 'oyaw': math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z),
                                 'ver': self.map_ver, 'frame': m.header.frame_id or 'map',
                                 'url': f'/api/map.png?v={self.map_ver}'}
        self.set('map', {'w_m': round(i.width * i.resolution, 1), 'h_m': round(i.height * i.resolution, 1),
                         'known_m2': round(float((known >= 0).sum()) * i.resolution ** 2, 1)}, '/map')

    def on_plan(self, m):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        with self.lock:
            self.plan = [[round(x, 3), round(y, 3)] for x, y in pts[::max(1, len(pts) // 300)]]

    def on_amcl(self, m):
        c = m.pose.covariance
        self.amcl = {'sx': math.sqrt(max(c[0], 0)), 'sy': math.sqrt(max(c[7], 0)),
                     'syaw': math.sqrt(max(c[35], 0)), 't': time.time()}
        self.rate['/amcl_pose'].tick()

    def on_particles(self, m):
        ps = m.particles[::max(1, len(m.particles) // 250)]
        with self.lock:
            self.particles = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)] for p in ps]

    def localization(self):
        """AMCL confidence for the dashboard: std-dev of position/heading -> good/fair/poor."""
        a = self.amcl
        if a is None:
            return None
        pos = math.hypot(a['sx'], a['sy'])
        q = 'good' if pos < 0.15 and a['syaw'] < 0.15 else 'fair' if pos < 0.4 and a['syaw'] < 0.4 else 'poor'
        return {'quality': q, 'pos_std_m': round(pos, 2), 'yaw_std_deg': round(math.degrees(a['syaw']), 1),
                'age_s': round(time.time() - a['t'], 1)}

    def on_gtsam_status(self, m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        self.set('gtsam', {k: s.get(k) for k in ('state', 'poses', 'overlap', 'rmse_m', 'processing_ms',
                                                 'gyro_bias_rad_s', 'use_imu', 'anchor', 'pose', 'detail')},
                 '/gtsam/status')

    def on_gtsam_path(self, m):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        with self.lock:
            self.gtsam_frame = m.header.frame_id or 'gtsam_odom'
            self.gtsam_path = pts[::max(1, len(pts) // 600)] + pts[-1:]
        self.rate['/gtsam/path'].tick()

    def gtsam_overlay(self, target='map'):
        """GTSAM path + pose in `target` (via the static TF target -> gtsam_odom the node publishes)."""
        with self.lock:
            pts, frame, st = list(self.gtsam_path), self.gtsam_frame, (self.v.get('gtsam') or {})
        if not st:
            return None
        out = {'state': st.get('state'), 'poses': st.get('poses'), 'anchor': st.get('anchor'),
               'path': [], 'robot': None, 'frame': target}
        T = self._lookup(target, frame)
        if T is None:
            return out
        c, s_ = math.cos(T[2]), math.sin(T[2])
        out['path'] = [[round(T[0] + c * x - s_ * y, 3), round(T[1] + s_ * x + c * y, 3)] for x, y in pts]
        p = st.get('pose')
        if p:
            out['robot'] = {'x': T[0] + c * p[0] - s_ * p[1], 'y': T[1] + s_ * p[0] + c * p[1], 'yaw': T[2] + p[2]}
        return out

    def reanchor_gtsam(self):
        if not self.gtsam_reanchor.wait_for_service(timeout_sec=1.0):
            return False, 'GTSAM is not running.'
        fut = self.gtsam_reanchor.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            time.sleep(0.05)
        if not fut.done():
            return False, 'GTSAM did not answer.'
        r = fut.result()
        return r.success, r.message

    # ---------- map-frame helpers ----------
    def _lookup(self, target, source):
        try:
            t = self.tf.lookup_transform(target, source, Time())
        except Exception:  # noqa: BLE001
            return None
        q, p = t.transform.rotation, t.transform.translation
        return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def robot_pose(self, frame='map'):
        return self._lookup(frame, 'base_footprint')

    def scan_in_map(self, step=4, frame='map'):
        m = self.last_scan
        if m is None:
            return []
        T = self._lookup(frame, m.header.frame_id)
        if T is None:
            return []
        r = np.asarray(m.ranges, dtype=np.float32)[::step]
        a = m.angle_min + np.arange(len(m.ranges))[::step] * m.angle_increment
        ok = np.isfinite(r) & (r > max(m.range_min, 0.12)) & (r < m.range_max)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        c, s_ = math.cos(T[2]), math.sin(T[2])
        return np.round(np.stack([T[0] + c * x - s_ * y, T[1] + s_ * x + c * y], 1), 3).tolist()

    @staticmethod
    def _pose(x, y, yaw, frame='map'):
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return ps

    def send_goal(self, x, y, yaw, label='goal'):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            return False, 'Navigation is still starting (takes ~1-2 min on the Jetson Nano). Try again shortly.'
        if self.robot_pose() is None:
            return False, ('Robot position on the map is not known yet. Wait a few seconds, or use '
                           '"Set pose" to show where the robot is.')
        g = NavigateToPose.Goal()
        g.pose = self._pose(x, y, yaw)
        g.pose.header.stamp = self.get_clock().now().to_msg()
        self.goal = {'x': x, 'y': y, 'yaw': yaw, 'label': label}
        self.nav_state = {'state': 'sending', 'label': label}
        fut = self.nav.send_goal_async(g, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_accepted)
        return True, f'Going to {label} ({x:.2f}, {y:.2f})'

    def _on_accepted(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.nav_state = {'state': 'rejected', 'label': self.goal and self.goal['label']}
            return
        self.goal_handle = gh
        self.nav_state = {'state': 'driving', 'label': self.goal and self.goal['label']}
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, fb):
        f = fb.feedback
        self.nav_state = {'state': 'driving', 'label': self.goal and self.goal['label'],
                          'remaining_m': round(f.distance_remaining, 2),
                          'eta_s': f.estimated_time_remaining.sec, 'recoveries': f.number_of_recoveries}

    def _on_result(self, fut):
        st = fut.result().status   # 4 = succeeded, 5 = canceled, 6 = aborted
        self.nav_state = {'state': {4: 'arrived', 5: 'canceled', 6: 'failed'}.get(st, f'status {st}'),
                          'label': self.goal and self.goal['label']}
        self.goal_handle = None

    def cancel_all_goals(self):
        """E-STOP: cancel every NavigateToPose goal, the patrol's too (empty request = all goals)."""
        if self.nav_cancel_all.service_is_ready():
            self.nav_cancel_all.call_async(CancelGoal.Request())
        self.goal = None

    def cancel_goal(self):
        if self.goal_handle is None:
            return False, 'No active goal.'
        self.goal_handle.cancel_goal_async()
        self.goal = None
        return True, 'Goal cancelled - robot stops.'

    def set_initial_pose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose = self._pose(x, y, yaw).pose
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05            # +-0.22 m: you clicked roughly the right spot
        cov[35] = 0.03                    # +-10 deg
        m.pose.covariance = cov
        for _ in range(3):
            self.initpose_pub.publish(m)
            time.sleep(0.05)
        # AMCL normally waits for the robot to move 10 cm before it re-matches the lidar;
        # ask it to update now (a few times) so the particles snap to the walls right away.
        if self.amcl_nomotion.service_is_ready():
            for _ in range(5):
                time.sleep(0.2)
                self.amcl_nomotion.call_async(Empty.Request())
        return True, f'Pose set to ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)'

    def snapshot(self):
        with self.lock:
            v = dict(self.v)
        rates = {t: self.rate[t].hz() for t in list(self.rate)}
        return v, rates

    def grab_cloud(self, timeout=20.0):
        """Wait for one /cloud_map message (published by RTAB-Map with map_3d:=true)."""
        got = []
        sub = self.create_subscription(PointCloud2, '/cloud_map', got.append, 1)
        t0 = time.time()
        while not got and time.time() - t0 < timeout:
            time.sleep(0.2)
        self.destroy_subscription(sub)
        return got[0] if got else None


# Robot programs our launches start. Some (e.g. Hiwonder's odom_publisher) ignore Ctrl+C and
# survive as orphans, which then block the next start ("/odom already publishing").
# Matched against the PROGRAM (argv[0], or the script for python) - never the whole command line,
# so shells/editors that merely mention these paths are never touched.
LEFTOVER_PROG = re.compile(
    r'(^/home/ubuntu/ros2_ws/install/'
    r'|^/opt/ros/humble/lib/(robot_state_publisher|joint_state_publisher|robot_localization|'
    r'imu_complementary_filter|laser_filters|sllidar_ros2|rtabmap_\w+|nav2_\w+|rclcpp_components|joy|slam_toolbox)/'
    r'|^/home/ubuntu/third_party_ros2/'          # sllidar_node (lidar driver) lives here
    r'|astra_bridge\.py$'
    r'|/lib/ame505_bringup/(xbox_teleop|checkpoint_recorder|patrol|frame_recorder|depth_obstacles|gtsam_slam|lifecycle_guard|sim_robot)$)')


def _leftover_pids():
    me, out = os.getpid(), []
    for d in os.listdir('/proc'):
        if not d.isdigit() or int(d) == me:
            continue
        try:
            argv = open(f'/proc/{d}/cmdline', 'rb').read().split(b'\0')
        except OSError:
            continue
        argv = [a.decode(errors='replace') for a in argv if a]
        if not argv:
            continue
        prog = argv[1] if os.path.basename(argv[0]).startswith('python') and len(argv) > 1 else argv[0]
        is_our_launch = (len(argv) > 3 and argv[1].endswith('/ros2') and argv[2] == 'launch'
                         and argv[3] == 'ame505_bringup')
        if is_our_launch or LEFTOVER_PROG.search(prog):
            out.append(int(d))
    return out


def kill_leftovers():
    """Kill orphaned robot processes (never the dashboard itself). Returns how many were found."""
    pids = _leftover_pids()
    found = len(pids)
    if not pids:
        return 0                                   # nothing to wait for (keeps E-STOP fast)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for p in pids:
            try:
                os.kill(p, sig)
            except ProcessLookupError:
                pass
        time.sleep(3 if sig != signal.SIGKILL else 0.5)
        pids = [p for p in pids if os.path.exists(f'/proc/{p}')]
        if not pids:
            break
    return found


class Manager:
    """Runs one ROS launch at a time (mapping or navigation) plus optional patrol."""

    def __init__(self, node):
        self.node = node
        self.proc = None
        self.patrol = None
        self.gtsam = None
        self.gtsam_check = {}
        self.mode = 'idle'
        self.stopping = False      # True while _stop_job runs: no second Stop, no Start
        self.kind = '2d'
        self.auto_pose = False
        self.map = None
        self.started = None
        self.log_path = None
        self.job = ''
        self.last_saved = {}
        self.lock = threading.Lock()

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def _spawn(self, cmd, tag):
        self.log_path = os.path.join(data_dir('logs'), f'{tag}_{time.strftime("%Y%m%d_%H%M%S")}.log')
        logf = open(self.log_path, 'w')
        return subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)

    def start(self, mode, name, new=False, rviz=False, map3d=False, camera=False):
        with self.lock:
            if self.stopping:
                return False, 'Still stopping the last run. Try again in a few seconds.'
            if self.running():
                return False, f'{self.mode} is already running. Stop it first.'
            if mode != 'drive' and not NAME_RE.match(name or ''):
                return False, 'Map name: letters, numbers, _ or - only.'
            # 2D (slam_toolbox / AMCL) unless 3D was asked for, or only an RTAB-Map db exists
            kind = '2d'
            if mode == 'mapping' and map3d:
                kind = '3d'
            if mode == 'navigation':
                if os.path.exists(map_yaml(name)):
                    kind = '2d'
                elif os.path.exists(map_db(name)):
                    kind = '3d'
                else:
                    return False, f'No saved map "{name}". Make it first with Start mapping.'
            v, rates = self.node.snapshot()
            if rates.get('/odom', 0) > 1:
                # leftover robot software from an earlier run -> clean it up instead of refusing
                kill_leftovers()
                board_stop()
                time.sleep(1)
            rv = f'use_rviz:={"true" if rviz else "false"}'
            cam = f'use_camera:={"true" if camera else "false"}'
            if mode == 'drive':
                cmd = ['ros2', 'launch', 'ame505_bringup', 'robot.launch.py', rv]
            elif kind == '2d':
                launch = 'mapping_2d.launch.py' if mode == 'mapping' else 'navigation_2d.launch.py'
                cmd = ['ros2', 'launch', 'ame505_bringup', launch, f'map:={name}', rv, cam]
            else:
                launch = 'mapping.launch.py' if mode == 'mapping' else 'navigation.launch.py'
                cmd = ['ros2', 'launch', 'ame505_bringup', launch, f'map:={name}', rv]
                if mode == 'mapping':
                    cmd.append(f'new:={"true" if new else "false"}')
            self.proc = self._spawn(cmd, mode)
            self.mode, self.map, self.started, self.kind = mode, name, time.time(), kind
            # AMCL's own start pose has ZERO uncertainty (all particles on one point), so it can't
            # correct a robot that isn't exactly at home. Once Nav2 is up, re-seed it at home with
            # realistic uncertainty and let the lidar pull it onto the walls.
            self.auto_pose = mode == 'navigation' and kind == '2d'
            with self.node.lock:                   # forget the previous run's map / AMCL state
                self.node.map_png, self.node.map_info, self.node.plan = None, None, []
                self.node.particles = []
                self.node.amcl = None
            if mode == 'drive':
                self.job = 'Starting... sensors appear in ~10 s. Then hold LB to drive.'
            elif mode == 'mapping':
                self.job = ('Starting 2D mapping... the map appears in ~15 s. Then drive slowly with LB.'
                            if kind == '2d' else 'Starting 3D mapping... RTAB-Map in ~15 s.')
            else:
                self.job = ('Starting navigation (2D map + AMCL)... ~1 min on the Nano.' if kind == '2d'
                            else 'Starting navigation (RTAB-Map 3D localization)... ~1-2 min.')
            return True, ' '.join(cmd)

    def _stop_proc(self, proc, grace=25):
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGINT)      # clean shutdown: RTAB-Map closes its database
            proc.wait(grace)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def stop(self, save):
        with self.lock:
            if self.stopping:
                return False, 'Already stopping.'
            if not self.running():
                self.mode = 'idle'
                # A restarted dashboard does not own the launch an earlier one started.
                if kill_leftovers():
                    return True, 'Stopped leftover robot software. ' + board_stop()[1]
                return False, 'Nothing is running.'
            self.stopping = True
        threading.Thread(target=self._stop_job, args=(save,), daemon=True).start()
        return True, 'Stopping...'

    def _stop_job(self, save):
        name, mode = self.map, self.mode
        saved = {'map': name}
        try:
            self._stop_proc(self.patrol, 5)
            self.stop_gtsam()
            kind = self.kind
            if save and mode == 'mapping' and kind == '3d':
                self.job = 'Saving 3D map (waiting for /cloud_map)...'
                msg = self.node.grab_cloud(20)
                if msg is not None:
                    path = save_cloud(msg, name)
                    saved['ply'] = os.path.basename(path) if path else None
            if save and mode == 'mapping':
                self.job = 'Saving 2D map...'
                out = os.path.join(data_dir('maps_2d'), name)
                subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', out,
                                '--ros-args', '-p', 'map_subscribe_transient_local:=true',
                                '-p', 'save_map_timeout:=10.0'],   # default 2 s fails on the busy Nano
                               timeout=40, capture_output=True)
                saved['pgm'] = os.path.exists(out + '.pgm') and os.path.getmtime(out + '.pgm') > time.time() - 60
            self.job = 'Stopping robot software (RTAB-Map writes its database)...'
            self.estop_twist()
            self._stop_proc(self.proc)
            kill_leftovers()                       # orphans would block the next start / the motor port
            # The motor board keeps its last speed when the driver dies -> zero it directly.
            ok, bmsg = board_stop()
            self.node.get_logger().info(bmsg)
            saved['db'] = kind == '3d' and os.path.exists(map_db(name))
            if save and mode == 'mapping' and saved['db']:
                self.job = 'Building realistic 3D scene from all recorded camera frames (1-5 min)...'
                ok, rmsg = self.reconstruct(name)
                saved['dense'] = f'{name}_mesh.ply' if ok else None
            self.last_saved = saved
            self.job = ('Saved: ' + ', '.join(
                [f'database {name}.db'] * bool(saved.get('db'))
                + [f'3D {saved["ply"]}'] * bool(saved.get('ply'))
                + [f'2D {name}.pgm'] * bool(saved.get('pgm'))
                + [f'realistic 3D {saved["dense"]}'] * bool(saved.get('dense')))) if save and mode == 'mapping' \
                else 'Stopped. ' + bmsg
            if save and mode == 'mapping' and kind == '3d' and not saved.get('ply'):
                self.job += ' (no 3D cloud received - was the camera running?)'
            if save and mode == 'mapping' and kind == '2d':
                self.job = (f'Saved 2D map "{name}" (maps_2d/{name}.pgm). Pick it in the Map panel and '
                            'press Start navigation.' if saved['pgm'] else
                            'Could not save the 2D map (no /map received?). See the log.')
        except Exception as e:  # noqa: BLE001
            self.job = f'Error while stopping: {e}'
        finally:
            self.mode = 'idle'
            self.stopping = False

    def reconstruct(self, name):
        try:
            r = subprocess.run(['ros2', 'run', 'ame505_bringup', 'reconstruct_3d', '--map', name],
                               capture_output=True, text=True, timeout=1200)
            out = (r.stdout + r.stderr).strip().splitlines()
            return r.returncode == 0, (out[-1] if out else 'no output')
        except subprocess.TimeoutExpired:
            return False, 'reconstruction timed out'

    def reconstruct_async(self, name):
        if not NAME_RE.match(name or '') or not os.path.exists(map_db(name)):
            return False, f'No saved map "{name}".'

        def job():
            self.job = f'Building realistic 3D scene for {name} (1-5 min)...'
            ok, msg = self.reconstruct(name)
            self.job = (f'Realistic 3D ready: {name}_mesh.ply - press "Open 3D map viewer"' if ok
                        else f'3D build failed: {msg}')
        threading.Thread(target=job, daemon=True).start()
        return True, f'Building realistic 3D scene for {name}...'

    def start_patrol(self, loops, dwell):
        if self.mode != 'navigation' or not self.running():
            return False, 'Start navigation first.'
        if self.patrol is not None and self.patrol.poll() is None:
            return False, 'Patrol already running.'
        self.patrol = self._spawn(['ros2', 'run', 'ame505_bringup', 'patrol', '--map', self.map,
                                   '--loops', str(loops), '--dwell', str(dwell)], 'patrol')
        return True, 'Patrol started.'

    # ---------- GTSAM (read-only estimator next to the running robot software) ----------
    def gtsam_running(self):
        return self.gtsam is not None and self.gtsam.poll() is None

    def start_gtsam(self, use_imu=True):
        if not self.running():
            return False, 'Start driving, mapping or navigation first (GTSAM needs the lidar/IMU).'
        if self.gtsam_running():
            return False, 'GTSAM is already running.'
        with self.node.lock:
            self.node.v.pop('gtsam', None)
            self.node.gtsam_path = []
        self.gtsam = self._spawn(['ros2', 'launch', 'ame505_bringup', 'gtsam_slam.launch.py',
                                  f'use_imu:={"true" if use_imu else "false"}'], 'gtsam')
        return True, ('GTSAM started - keep the robot STILL for ~5 s (gyro calibration), '
                      'then drive. Orange path on the map.' if use_imu else
                      'GTSAM started (lidar only). Orange path on the map.')

    def stop_gtsam(self):
        if not self.gtsam_running():
            return False, 'GTSAM is not running.'
        self._stop_proc(self.gtsam, 5)
        with self.node.lock:
            self.node.v.pop('gtsam', None)
        return True, 'GTSAM stopped (trajectory saved under gtsam/ on the SSD).'

    def gtsam_check_async(self, name):
        if not NAME_RE.match(name or '') or not os.path.exists(map_db(name)):
            return False, f'No saved map "{name}".'

        def job():
            self.job = f'Checking GTSAM on saved map {name}...'
            try:
                r = subprocess.run(['ros2', 'run', 'ame505_bringup', 'gtsam_map_check', '--map', name],
                                   capture_output=True, text=True, timeout=300)
                lines = (r.stdout + r.stderr).strip().splitlines()
                ok = r.returncode == 0 and os.path.exists(os.path.join(data_dir('gtsam'), f'{name}_check.png'))
                self.gtsam_check = {'map': name, 'ok': ok, 'text': '\n'.join(lines[-3:]),
                                    'png': f'/files/gtsam/{name}_check.png?t={int(time.time())}' if ok else None}
                self.job = (lines[1].strip() if ok and len(lines) > 1 else f'GTSAM check failed: {lines[-1] if lines else "no output"}')
            except subprocess.TimeoutExpired:
                self.job = 'GTSAM check timed out.'
        threading.Thread(target=job, daemon=True).start()
        return True, f'Checking GTSAM on saved map {name}...'

    def estop_twist(self):
        for _ in range(5):
            self.node.estop_pub.publish(Twist())
            time.sleep(0.02)

    def estop(self):
        self.node.cancel_all_goals()       # Nav2 then commands zero itself, before anything is killed
        self.estop_twist()
        self._stop_proc(self.patrol, 3)
        if not self.running():
            kill_leftovers()               # an orphaned driver would keep the motor port busy
            ok, bmsg = board_stop()        # nothing running: talk to the motor board directly
            return True, 'E-STOP: ' + bmsg
        if self.mode in ('navigation', 'drive'):
            self.stop(save=False)          # stop job also zeroes the motor board
        return True, 'E-STOP: zero velocity sent' + (f', {self.mode} stopping.' if self.mode in ('navigation', 'drive') else '.')

    def log_tail(self, n=25):
        if not self.log_path or not os.path.exists(self.log_path):
            return ''
        with open(self.log_path, 'rb') as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 6000))
            lines = f.read().decode(errors='replace').splitlines()[-n:]
        return '\n'.join(re.sub(r'\x1b\[[0-9;]*m', '', ln)[:220] for ln in lines)


def save_cloud(msg, name):
    from sensor_msgs_py import point_cloud2
    fields = [f.name for f in msg.fields]
    has_rgb = 'rgb' in fields
    st = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'] + (['rgb'] if has_rgb else []),
                                  skip_nans=True)
    if len(st) == 0:
        return None
    xyz = np.stack([st['x'], st['y'], st['z']], axis=1).astype(np.float32)
    rgb = None
    if has_rgb:
        packed = np.asarray(st['rgb'], dtype=np.float32).view(np.uint32)
        rgb = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], 1).astype(np.uint8)
    path = os.path.join(data_dir('maps_3d'), f'{name}_{time.strftime("%Y%m%d_%H%M%S")}.ply')
    write_ply(path, xyz, rgb)
    return path


_saved_cache = {}


def saved_grid_2d(name):
    """maps_2d/<name>.pgm + .yaml (map_saver) -> (info, png bytes)."""
    yml = map_yaml(name)
    try:
        meta = yaml.safe_load(open(yml))
        pgm = os.path.join(os.path.dirname(yml), meta['image'])
        mt = os.path.getmtime(pgm)
        key = '2d:' + name
        if key in _saved_cache and _saved_cache[key][0] == mt:
            return _saved_cache[key][1:]
        raw = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
    except Exception:  # noqa: BLE001
        return None, None
    if raw is None:
        return None, None
    neg = bool(meta.get('negate', 0))
    occ = (raw / 255.0) if neg else (255 - raw) / 255.0          # occupancy probability
    img = np.full(raw.shape, 128, np.uint8)
    img[occ < meta.get('free_thresh', 0.25)] = 235
    img[occ > meta.get('occupied_thresh', 0.65)] = 0
    png = cv2.imencode('.png', img)[1].tobytes()                  # pgm row 0 is already the top
    o = meta.get('origin', [0, 0, 0])
    info = {'res': float(meta['resolution']), 'w': int(raw.shape[1]), 'h': int(raw.shape[0]),
            'ox': float(o[0]), 'oy': float(o[1]), 'oyaw': float(o[2]), 'frame': 'map',
            'url': f'/api/saved_map.png?name={name}&t={int(mt)}'}
    _saved_cache['2d:' + name] = (mt, info, png)
    return info, png


def saved_grid(name):
    """Saved map preview: the 2D map if there is one, else the grid inside the RTAB-Map db."""
    import sqlite3
    if not NAME_RE.match(name or ''):
        return None, None
    if os.path.exists(map_yaml(name)):
        return saved_grid_2d(name)
    db = map_db(name)
    if not os.path.exists(db):
        return None, None
    mt = os.path.getmtime(db)
    if name in _saved_cache and _saved_cache[name][0] == mt:
        return _saved_cache[name][1:]
    try:
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        b, x0, y0, res = con.execute(
            'select opt_map, opt_map_x_min, opt_map_y_min, opt_map_resolution from Admin').fetchone()
        con.close()
        if not b:
            return None, None
        g = mat_uncompress(b)
    except Exception:  # noqa: BLE001  (database still open by RTAB-Map, or no grid yet)
        return None, None
    img = np.full(g.shape, 128, np.uint8)
    img[g == 0] = 235
    img[g > 0] = 0
    png = cv2.imencode('.png', img[::-1])[1].tobytes()
    info = {'res': float(res), 'w': int(g.shape[1]), 'h': int(g.shape[0]), 'ox': float(x0), 'oy': float(y0),
            'oyaw': 0.0, 'frame': 'map', 'url': f'/api/saved_map.png?name={name}&t={int(mt)}'}
    _saved_cache[name] = (mt, info, png)
    return info, png


def home_file(name):
    return os.path.join(data_dir('home'), f'{name}.yaml')


def get_home(name):
    """Home = where you saved it with "Set home here"; default = map origin = where mapping started."""
    p = home_file(name) if name else None
    if p and os.path.exists(p):
        h = yaml.safe_load(open(p)) or {}
        return {'x': h.get('x', 0.0), 'y': h.get('y', 0.0), 'yaw': h.get('yaw', 0.0), 'source': 'saved'}
    return {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'source': 'mapping start'}


def load_checkpoints(name):
    p = checkpoints_file(name) if name else None
    if p and os.path.exists(p):
        return (yaml.safe_load(open(p)) or {}).get('checkpoints', [])
    return []


def list_files():
    def info(p):
        return {'name': os.path.basename(p), 'mb': round(os.path.getsize(p) / 1e6, 1),
                'time': time.strftime('%Y-%m-%d %H:%M', time.localtime(os.path.getmtime(p)))}
    by_time = lambda pat: sorted(glob.glob(pat), key=os.path.getmtime, reverse=True)  # noqa: E731
    maps, seen = [], set()
    for p in by_time(os.path.join(data_dir('maps_2d'), '*.yaml')) + by_time(os.path.join(data_dir('maps'), '*.db')):
        n = os.path.splitext(os.path.basename(p))[0]
        if n in seen:                          # 2D map wins over a 3D db with the same name
            continue
        seen.add(n)
        d = info(p)
        d.update({'map': n, 'kind': '2d' if p.endswith('.yaml') else '3d'})
        if d['kind'] == '2d':
            pgm = p[:-5] + '.pgm'
            d['mb'] = round(os.path.getsize(pgm) / 1e6, 2) if os.path.exists(pgm) else 0
        maps.append(d)
    return {'maps': maps,
            'clouds': [info(p) for p in by_time(os.path.join(data_dir('maps_3d'), '*.ply'))],
            'data_dir': data_dir()}


def create_app(node, mgr):
    app = Flask(__name__, static_folder=None)

    @app.get('/')
    def index():
        return send_from_directory(WEB, 'index.html')

    @app.get('/viewer')
    def viewer():
        return send_from_directory(WEB, 'viewer.html')

    @app.get('/api/status')
    def status():
        v, rates = node.snapshot()
        return jsonify({
            'mode': mgr.mode, 'map': mgr.map, 'running': mgr.running(), 'kind': mgr.kind,
            'uptime_s': int(time.time() - mgr.started) if mgr.running() and mgr.started else 0,
            'patrol': mgr.patrol is not None and mgr.patrol.poll() is None,
            'job': mgr.job, 'last_saved': mgr.last_saved, 'sensors': v, 'rates': rates,
            'log': mgr.log_tail(), 'gtsam_running': mgr.gtsam_running(), 'gtsam_check': mgr.gtsam_check,
        })

    @app.get('/api/scan')
    def scan():
        with node.lock:
            return jsonify(node.scan_pts)

    @app.get('/api/files')
    def files():
        return jsonify(list_files())

    @app.get('/api/map.png')
    def map_png():
        with node.lock:
            png = node.map_png
        if png is None:
            abort(404)
        return Response(png, mimetype='image/png', headers={'Cache-Control': 'no-store'})

    @app.get('/api/saved_map.png')
    def saved_map_png():
        info, png = saved_grid(request.args.get('name', ''))
        if png is None:
            abort(404)
        return Response(png, mimetype='image/png', headers={'Cache-Control': 'no-store'})

    @app.get('/api/navstate')
    def navstate():
        live = mgr.running() and mgr.mode in ('mapping', 'navigation')
        if live:                                   # live map + robot from the running software
            with node.lock:
                info, plan = node.map_info, list(node.plan)
            rp = node.robot_pose()
            name = mgr.map
            if info is None and mgr.mode == 'navigation':
                info, _ = saved_grid(name)         # show the saved map while RTAB-Map is still loading
            up = int(time.time() - mgr.started) if mgr.started else 0
            loc = node.localization() if mgr.kind == '2d' else None
            if mgr.mode == 'navigation':
                nav_ready = node.nav.server_is_ready()
                phase = ('ready' if rp and nav_ready else
                         'localizing' if nav_ready or up > 90 else 'starting')
                if phase == 'ready' and mgr.auto_pose:
                    mgr.auto_pose = False
                    h = get_home(name)
                    threading.Thread(target=node.set_initial_pose, args=(h['x'], h['y'], h['yaw']),
                                     daemon=True).start()
                if phase == 'ready' and loc and loc['quality'] == 'poor':
                    phase = 'lost'                # AMCL unsure: ask the user to Set pose
            else:
                phase = 'mapping' if rp else 'starting'
            return jsonify({
                'live': True, 'phase': phase, 'uptime_s': up,
                'map': info, 'mode': mgr.mode, 'map_name': name,
                'robot': {'x': rp[0], 'y': rp[1], 'yaw': rp[2]} if rp else None,
                'scan': node.scan_in_map(), 'plan': plan, 'goal': node.goal, 'nav': node.nav_state,
                'home': get_home(name) if name else None, 'checkpoints': load_checkpoints(name),
                'gtsam': node.gtsam_overlay('map') if mgr.gtsam_running() else None,
                'loc': loc, 'particles': list(node.particles) if mgr.mode == 'navigation' else [],
                'kind': mgr.kind,
            })
        if mgr.running() and mgr.mode == 'drive' and mgr.gtsam_running():
            # Just driving (no map): show GTSAM vs wheel/EKF odometry live in the odom frame.
            rp = node.robot_pose('odom')
            return jsonify({
                'live': True, 'phase': 'drive', 'uptime_s': 0, 'map': None, 'mode': mgr.mode,
                'map_name': None, 'robot': {'x': rp[0], 'y': rp[1], 'yaw': rp[2]} if rp else None,
                'scan': node.scan_in_map(frame='odom'), 'plan': [], 'goal': None, 'nav': {'state': 'idle'},
                'home': None, 'checkpoints': [], 'gtsam': node.gtsam_overlay('odom'),
            })
        name = request.args.get('map', '')       # preview of the selected saved map
        info, _ = saved_grid(name)
        return jsonify({
            'live': False, 'map': info, 'mode': mgr.mode, 'map_name': name, 'robot': None, 'scan': [],
            'plan': [], 'goal': None, 'nav': {'state': 'idle'},
            'home': get_home(name) if info else None, 'checkpoints': load_checkpoints(name) if info else [],
        })

    @app.post('/api/<action>')
    def action(action):
        body = request.get_json(silent=True) or {}
        if action == 'start_drive':
            ok, msg = mgr.start('drive', None, rviz=bool(body.get('rviz')))
        elif action == 'start_mapping':
            ok, msg = mgr.start('mapping', body.get('map', 'lab2d'), bool(body.get('new')), bool(body.get('rviz')),
                                map3d=bool(body.get('map3d')), camera=bool(body.get('camera')))
        elif action == 'start_navigation':
            ok, msg = mgr.start('navigation', body.get('map', 'lab2d'), rviz=bool(body.get('rviz')),
                                camera=bool(body.get('camera')))
        elif action == 'stop_save':
            ok, msg = mgr.stop(save=True)
        elif action == 'stop':
            ok, msg = mgr.stop(save=False)
        elif action == 'estop':
            ok, msg = mgr.estop()
        elif action in ('goal', 'go_home', 'go_checkpoint'):
            if mgr.mode != 'navigation' or not mgr.running():
                ok, msg = False, 'Start navigation first (Navigate & patrol panel).'
            elif action == 'goal':
                ok, msg = node.send_goal(float(body['x']), float(body['y']), float(body.get('yaw', 0)))
            elif action == 'go_home':
                h = get_home(mgr.map)
                ok, msg = node.send_goal(h['x'], h['y'], h['yaw'], 'home')
            else:
                cps = {c['id']: c for c in load_checkpoints(mgr.map)}
                c = cps.get(body.get('id'))
                ok, msg = (node.send_goal(c['x'], c['y'], c['yaw'], c['id']) if c
                           else (False, 'Unknown checkpoint'))
        elif action == 'cancel_goal':
            ok, msg = node.cancel_goal()
            mgr.estop_twist()
        elif action == 'initialpose':
            if mgr.mode != 'navigation' or not mgr.running():
                ok, msg = False, 'Start navigation first.'
            else:
                ok, msg = node.set_initial_pose(float(body['x']), float(body['y']), float(body.get('yaw', 0)))
        elif action in ('set_home', 'add_checkpoint'):
            rp = node.robot_pose()
            if not mgr.map or rp is None:
                ok, msg = False, 'Robot position on the map is unknown (start navigation/mapping first).'
            elif action == 'set_home':
                with open(home_file(mgr.map), 'w') as f:
                    yaml.safe_dump({'x': round(rp[0], 3), 'y': round(rp[1], 3), 'yaw': round(rp[2], 3)}, f)
                ok, msg = True, f'Home set here ({rp[0]:.2f}, {rp[1]:.2f}).'
            else:
                cps = load_checkpoints(mgr.map)
                cps.append({'id': f'cp_{len(cps) + 1:02d}', 'x': round(rp[0], 3), 'y': round(rp[1], 3),
                            'yaw': round(rp[2], 3)})
                with open(checkpoints_file(mgr.map), 'w') as f:
                    yaml.safe_dump({'frame_id': 'map', 'checkpoints': cps}, f, sort_keys=False)
                ok, msg = True, f'Checkpoint {cps[-1]["id"]} saved.'
        elif action == 'reconstruct':
            ok, msg = mgr.reconstruct_async(body.get('map', ''))
        elif action == 'patrol':
            ok, msg = mgr.start_patrol(int(body.get('loops', 1)), float(body.get('dwell', 5)))
        elif action == 'start_gtsam':
            ok, msg = mgr.start_gtsam(bool(body.get('use_imu', True)))
        elif action == 'stop_gtsam':
            ok, msg = mgr.stop_gtsam()
        elif action == 'gtsam_reanchor':
            ok, msg = node.reanchor_gtsam()
        elif action == 'gtsam_check':
            ok, msg = mgr.gtsam_check_async(body.get('map', ''))
        else:
            abort(404)
        if ok:
            mgr.job = msg if action not in ('start_drive', 'start_mapping', 'start_navigation', 'stop', 'stop_save') else mgr.job
        return jsonify({'ok': ok, 'msg': msg}), (200 if ok else 400)

    @app.get('/files/maps_3d/<path:name>')
    def ply(name):
        return send_from_directory(data_dir('maps_3d'), name)

    @app.get('/files/gtsam/<path:name>')
    def gtsam_file(name):
        return send_from_directory(data_dir('gtsam'), name)

    blank = np.zeros((240, 320, 3), np.uint8)
    cv2.putText(blank, 'no signal - start mapping', (22, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (140, 140, 140), 1)
    placeholder = cv2.imencode('.jpg', blank)[1].tobytes()

    def mjpeg(kind):
        def part(jpg):
            return b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n'

        def gen():
            # Send something immediately: otherwise the browser never gets the response
            # headers while the camera is off and the tab keeps "loading" forever.
            yield part(placeholder)
            last, idle = -1, 0.0
            while True:
                with node.lock:
                    fid, jpg = node.frame_id[kind], node.jpeg[kind]
                if jpg is not None and fid != last:
                    last, idle = fid, 0.0
                    yield part(jpg)
                elif idle > 2.0:            # keep-alive / back to placeholder when camera stops
                    idle = 0.0
                    yield part(placeholder if jpg is None else jpg)
                time.sleep(0.05)
                idle += 0.05
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.get('/stream/<kind>')
    def stream(kind):
        if kind not in ('rgb', 'depth'):
            abort(404)
        return mjpeg(kind)

    return app


def main():
    rclpy.init()
    node = Sensors()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    mgr = Manager(node)
    app = create_app(node, mgr)
    # `dashboard.sh restart` escalates to SIGTERM: exit through the `finally` below so the
    # running launch is stopped instead of being left behind as an orphan.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    print(f'\n  AME 505 dashboard:  http://localhost:{PORT}   (laptop: http://<robot-ip>:{PORT})\n', flush=True)
    try:
        app.run(host='0.0.0.0', port=PORT, threaded=True, use_reloader=False)
    finally:
        if mgr.running():
            print('Stopping running launch (Ctrl+C)...', flush=True)
            mgr._stop_proc(mgr.patrol, 3)
            mgr._stop_proc(mgr.gtsam, 3)
            mgr._stop_proc(mgr.proc)
            print(board_stop()[1], flush=True)
        ex.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
