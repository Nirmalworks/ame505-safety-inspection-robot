"""ROS-free, experimental planar lidar/gyro odometry for the JetAuto."""

from collections import deque
from dataclasses import dataclass
from functools import partial
import math

import gtsam
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Config:
    use_imu: bool = True
    calibration_seconds: float = 4.0
    calibration_max_gyro_std: float = 0.02
    calibration_max_gyro_mean: float = 0.05
    calibration_max_translation: float = 0.025
    calibration_max_yaw: float = 0.03
    max_imu_gap: float = 0.10
    max_scan_gap: float = 1.0
    min_range: float = 0.15
    max_range: float = 6.0
    max_points: int = 300
    min_points: int = 40
    correspondence_distance: float = 0.20
    icp_iterations: int = 35
    min_overlap: float = 0.65
    max_rmse: float = 0.06
    min_geometry_ratio: float = 0.001
    max_speed: float = 0.8
    max_yaw_rate: float = 1.5
    motion_slack: float = 0.03
    lidar_translation_sigma: float = 0.03
    lidar_yaw_sigma: float = 0.05
    gyro_noise_density: float = 0.01
    gyro_bias_sigma: float = 0.003
    max_poses: int = 3000

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == 'use_imu':
                if not isinstance(value, bool):
                    raise ValueError('use_imu must be boolean')
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if self.max_points < self.min_points or self.min_points < 5:
            raise ValueError('Require max_points >= min_points >= 5')
        if self.min_range >= self.max_range or self.min_overlap > 1:
            raise ValueError('Invalid range or overlap limits')
        for name in ('max_points', 'min_points', 'icp_iterations', 'max_poses'):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f'{name} must be an integer')


def rotation_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid TF quaternion')
    return Rotation.from_quat(q).as_matrix()


def scan_points(ranges, angle_min, angle_increment, range_min, range_max,
                translation, quaternion, config):
    """Transform valid laser returns into base-frame XY using real sensor TF."""
    rotation = rotation_matrix(quaternion)
    # This planar estimator requires a horizontal scan plane (either facing up or down).
    if abs(rotation[2, 2]) < math.cos(math.radians(5)):
        raise ValueError('Lidar scan plane is tilted by more than 5 degrees')
    if not all(math.isfinite(x) for x in (*translation, angle_min, angle_increment,
                                         range_min, range_max)) or angle_increment == 0:
        raise ValueError('Invalid scan metadata or translation')
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(len(r)) * angle_increment
    keep = np.isfinite(r) & (r >= max(range_min, config.min_range)) & (
        r <= min(range_max, config.max_range))
    points = np.column_stack((r[keep] * np.cos(angles[keep]),
                              r[keep] * np.sin(angles[keep]), np.zeros(np.sum(keep))))
    if len(points) > config.max_points:
        points = points[np.linspace(0, len(points) - 1, config.max_points, dtype=int)]
    return (points @ rotation.T + np.asarray(translation))[:, :2]


def transformed(points, pose):
    c, s = math.cos(pose.theta()), math.sin(pose.theta())
    return points @ np.array([[c, s], [-s, c]]) + [pose.x(), pose.y()]


def scan_match(previous, current, guess, config):
    """Point-to-line ICP: return T_previous_current with conservative quality gates."""
    if min(len(previous), len(current)) < config.min_points:
        raise ValueError('too_few_scan_points')
    tree = cKDTree(previous)
    _, neighbors = tree.query(previous, k=5)
    local = previous[neighbors]
    local -= local.mean(axis=1, keepdims=True)
    _, vectors = np.linalg.eigh(np.einsum('nki,nkj->nij', local, local))
    normals = vectors[:, :, 0]
    pose = guess
    converged = False
    for _ in range(config.icp_iterations):
        moved = transformed(current, pose)
        distance, index = tree.query(moved)
        keep = distance < config.correspondence_distance
        if np.sum(keep) < config.min_points:
            raise ValueError('too_few_correspondences')
        # Trim the worst 15% of correspondences during the solve, then score all matches.
        keep &= distance <= np.quantile(distance[keep], 0.85)
        source, target = moved[keep], previous[index[keep]]
        normal = normals[index[keep]]
        jac = np.column_stack((normal, -normal[:, 0] * source[:, 1] +
                               normal[:, 1] * source[:, 0]))
        residual = np.sum(normal * (source - target), axis=1)
        step = np.linalg.lstsq(jac, -residual, rcond=None)[0]
        delta = gtsam.Pose2(*step)
        pose = delta.compose(pose)
        if np.linalg.norm(step[:2]) < 1e-4 and abs(delta.theta()) < 1e-4:
            converged = True
            break
    if not converged:
        raise ValueError('icp_not_converged')
    moved = transformed(current, pose)
    distance, index = tree.query(moved)
    keep = distance < config.correspondence_distance
    overlap = float(np.mean(keep))
    if np.sum(keep) < config.min_points or overlap < config.min_overlap:
        raise ValueError('low_scan_overlap')
    rmse = float(np.sqrt(np.mean(distance[keep] ** 2)))
    if rmse > config.max_rmse:
        raise ValueError('high_scan_rmse')
    # Local surface normals expose unobservable motion along a lone wall/corridor.
    normal = normals[index[keep]]
    p = moved[keep]
    scale = max(float(np.median(np.linalg.norm(p, axis=1))), 0.5)
    jac = np.column_stack((normal, (-normal[:, 0] * p[:, 1] +
                                    normal[:, 1] * p[:, 0]) / scale))
    eigen = np.linalg.eigvalsh(jac.T @ jac)
    ratio = float(eigen[0] / max(eigen[-1], 1e-12))
    if ratio < config.min_geometry_ratio:
        raise ValueError('degenerate_scan_geometry')
    return pose, {'overlap': overlap, 'rmse_m': rmse, 'geometry_ratio': ratio}


class GyroBuffer:
    def __init__(self, max_gap):
        self.samples = deque(maxlen=20000)
        self.max_gap = max_gap

    def add(self, stamp, rate):
        if not math.isfinite(stamp) or not math.isfinite(rate):
            raise ValueError('nonfinite_imu')
        if self.samples and stamp <= self.samples[-1][0]:
            raise ValueError('imu_time_not_increasing')
        self.samples.append((stamp, rate))

    def interval(self, start, end):
        data = np.asarray(self.samples)
        if len(data) < 2 or data[0, 0] > start or data[-1, 0] < end:
            raise ValueError('waiting_for_imu_coverage')
        left = max(int(np.searchsorted(data[:, 0], start, side='right')) - 1, 0)
        right = min(int(np.searchsorted(data[:, 0], end, side='left')) + 1, len(data))
        data = data[left:right]
        if np.max(np.diff(data[:, 0])) > self.max_gap:
            raise ValueError('imu_gap')
        inside = data[(data[:, 0] > start) & (data[:, 0] < end)]
        times = np.r_[start, inside[:, 0], end]
        rates = np.r_[np.interp(start, data[:, 0], data[:, 1]), inside[:, 1],
                      np.interp(end, data[:, 0], data[:, 1])]
        return times, rates

    def integrate(self, start, end, bias):
        times, rates = self.interval(start, end)
        return float(np.trapz(rates - bias, times))


def gyro_error(measurement, this, values, jacobians):
    i, j = this.keys()
    residual = wrap(values.atPose2(j).theta() - values.atPose2(i).theta() - measurement)
    if jacobians is not None:
        jacobians[0] = np.array([[0., 0., -1.]], order='F')
        jacobians[1] = np.array([[0., 0., 1.]], order='F')
    return np.array([residual])


class IncrementalGraph:
    def __init__(self, config):
        self.config = config
        self.isam = gtsam.ISAM2()
        graph, initial = gtsam.NonlinearFactorGraph(), gtsam.Values()
        graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(),
                  gtsam.noiseModel.Diagonal.Sigmas(np.full(3, 1e-4))))
        initial.insert(0, gtsam.Pose2())
        self.isam.update(graph, initial)
        self.count = 1
        self.pose = gtsam.Pose2()
        # Keep Python callback references alive for GTSAM's CustomFactors.
        self.callbacks = []

    def append(self, motion, gyro_yaw, dt):
        if self.count >= self.config.max_poses:
            raise ValueError('session_pose_limit_restart_required')
        i, j = self.count - 1, self.count
        graph, initial = gtsam.NonlinearFactorGraph(), gtsam.Values()
        sigmas = np.array([self.config.lidar_translation_sigma] * 2 +
                          [self.config.lidar_yaw_sigma])
        graph.add(gtsam.BetweenFactorPose2(i, j, motion,
                  gtsam.noiseModel.Diagonal.Sigmas(sigmas)))
        if gyro_yaw is not None:
            sigma = math.hypot(self.config.gyro_noise_density * math.sqrt(dt),
                               self.config.gyro_bias_sigma * dt)
            callback = partial(gyro_error, gyro_yaw)
            self.callbacks.append(callback)
            graph.add(gtsam.CustomFactor(gtsam.noiseModel.Isotropic.Sigma(1, sigma),
                                         [i, j], callback))
        initial.insert(j, self.pose.compose(motion))
        self.isam.update(graph, initial)
        self.pose = self.isam.calculateEstimatePose2(j)
        self.count += 1
        return self.pose

    def covariance(self):
        return self.isam.marginalCovariance(self.count - 1)


class HardwareOdometry:
    def __init__(self, config):
        self.config = config
        self.gyro = GyroBuffer(config.max_imu_gap)
        self.graph = None
        self.reference = None
        self.stamp = None
        self.calibration_start = None
        self.calibration_reference = None
        self.bias = 0.0
        self.failed = None
        self.last_input_stamp = None

    def process(self, stamp, points):
        """Return (pose or None, status). Rejected scans never advance the trajectory."""
        if self.failed:
            return None, {'state': self.failed}
        if not math.isfinite(stamp):
            return None, {'state': 'invalid_scan_time'}
        if self.last_input_stamp is not None and stamp <= self.last_input_stamp:
            self.failed = 'scan_time_reset_restart_required'
            return None, {'state': self.failed}
        self.last_input_stamp = stamp
        try:
            if len(points) < self.config.min_points or not np.isfinite(points).all():
                raise ValueError('too_few_or_invalid_scan_points')
            if self.graph is None:
                if self.config.use_imu:
                    if self.calibration_start is None:
                        # Start only once IMU timestamps can cover the first scan.
                        if not self.gyro.samples or self.gyro.samples[0][0] > stamp:
                            raise ValueError('waiting_for_imu')
                        self.calibration_start = stamp
                        self.calibration_reference = points
                    delta, _ = scan_match(self.calibration_reference, points,
                                          gtsam.Pose2(), self.config)
                    if math.hypot(delta.x(), delta.y()) > self.config.calibration_max_translation or \
                            abs(delta.theta()) > self.config.calibration_max_yaw:
                        self.failed = 'moved_during_calibration_restart_required'
                        raise ValueError(self.failed)
                    if stamp - self.calibration_start < self.config.calibration_seconds:
                        return None, {'state': 'calibrating_keep_robot_still'}
                    _, rates = self.gyro.interval(self.calibration_start, stamp)
                    if np.std(rates) > self.config.calibration_max_gyro_std or \
                            abs(np.mean(rates)) > self.config.calibration_max_gyro_mean:
                        self.failed = 'gyro_calibration_failed_restart_required'
                        raise ValueError(self.failed)
                    self.bias = float(np.mean(rates))
                self.graph = IncrementalGraph(self.config)
                self.reference, self.stamp = points, stamp
                return self.graph.pose, {'state': 'tracking', 'gyro_bias_rad_s': self.bias,
                                         'poses': 1}
            dt = stamp - self.stamp
            if dt > self.config.max_scan_gap:
                self.failed = 'tracking_lost_restart_required'
                raise ValueError(self.failed)
            yaw = self.gyro.integrate(self.stamp, stamp, self.bias) if self.config.use_imu else None
            motion, quality = scan_match(self.reference, points,
                                         gtsam.Pose2(0, 0, yaw or 0), self.config)
            if math.hypot(motion.x(), motion.y()) > self.config.max_speed * dt + self.config.motion_slack:
                raise ValueError('implausible_translation')
            if abs(motion.theta()) > self.config.max_yaw_rate * dt + self.config.motion_slack:
                raise ValueError('implausible_rotation')
            if yaw is not None and abs(wrap(motion.theta() - yaw)) > max(0.10, 3 * self.config.lidar_yaw_sigma):
                raise ValueError('lidar_gyro_disagreement')
            pose = self.graph.append(motion, yaw, dt)
            self.reference, self.stamp = points, stamp
            return pose, {'state': 'tracking', 'poses': self.graph.count,
                          'gyro_bias_rad_s': self.bias, **quality}
        except ValueError as exc:
            return None, {'state': str(exc)}
