import math
import unittest

import gtsam
import numpy as np

from hardware_core import (Config, GyroBuffer, HardwareOdometry, IncrementalGraph,
                           gyro_error, scan_match, scan_points, transformed)
from synthetic_sensors import base_pose, room_scan


def room_points():
    """Asymmetric room surfaces, with a corner visible in every quadrant."""
    x = np.linspace(-2, 3, 100)
    y = np.linspace(-1.5, 2, 80)
    return np.vstack((np.column_stack((x, np.full_like(x, -1.5))),
                      np.column_stack((x, np.full_like(x, 2))),
                      np.column_stack((np.full_like(y, -2), y)),
                      np.column_stack((np.full_like(y, 3), y))))


class HardwareTests(unittest.TestCase):
    def test_end_to_end_raycast_scans_gyro_and_incremental_graph(self):
        config = Config(calibration_seconds=0.5)
        estimator = HardwareOdometry(config)
        next_imu = 0.
        accepted = 0
        for stamp in np.arange(0., 6.01, 0.2):
            while next_imu <= stamp + 0.02:
                estimator.gyro.add(next_imu, (0.04 if next_imu > 3. else 0.) + 0.001)
                next_imu += 0.02
            expected = base_pose(float(stamp))
            points = scan_points(room_scan(expected), -math.pi / 2, math.pi / 359,
                                 0.1, 10., [0.096, 0, 0], [0, 0, 1, 0], config)
            pose, status = estimator.process(float(stamp), points)
            if pose is not None:
                accepted += 1
                self.assertAlmostEqual(pose.x(), expected[0], delta=0.015)
                self.assertAlmostEqual(pose.y(), expected[1], delta=0.015)
                self.assertAlmostEqual(pose.theta(), expected[2], delta=0.01)
        self.assertEqual(status['state'], 'tracking')
        self.assertGreater(accepted, 20)

    def test_icp_transform_direction_and_mecanum_lateral_motion(self):
        previous = room_points()
        expected = gtsam.Pose2(0.035, 0.025, 0.02)
        current = transformed(previous, expected.inverse())
        result, quality = scan_match(previous, current, gtsam.Pose2(0, 0, 0.018), Config())
        np.testing.assert_allclose(result.matrix(), expected.matrix(), atol=2e-3)
        self.assertGreater(quality['overlap'], 0.95)

    def test_single_wall_is_rejected_as_unobservable(self):
        line = np.column_stack((np.linspace(-2, 2, 150), np.ones(150)))
        with self.assertRaisesRegex(ValueError, 'degenerate_scan_geometry'):
            scan_match(line, line, gtsam.Pose2(), Config())

    def test_reversed_lidar_extrinsic_and_invalid_ranges(self):
        result = scan_points([1., float('inf'), float('nan'), 0., 100.],
                             0., 0.1, 0.1, 10., [0.096, 0, 0.1],
                             [0, 0, 1, 0], Config())
        np.testing.assert_allclose(result, [[-0.904, 0]], atol=1e-10)

    def test_tilted_lidar_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'tilted'):
            scan_points([1.] * 60, 0., 0.1, 0.1, 10., [0, 0, 0],
                        [math.sin(0.2), 0, 0, math.cos(0.2)], Config())

    def test_gyro_interpolation_bias_and_gap(self):
        gyro = GyroBuffer(0.11)
        for t in np.arange(0, 1.01, 0.1):
            gyro.add(float(t), 0.201)
        self.assertAlmostEqual(gyro.integrate(0.05, 0.95, 0.001), 0.18)
        gyro.add(1.5, 0.201)
        with self.assertRaisesRegex(ValueError, 'imu_gap'):
            gyro.integrate(1., 1.5, 0.001)
        with self.assertRaisesRegex(ValueError, 'coverage'):
            gyro.integrate(1., 1.6, 0.001)

    def test_gyro_factor_jacobians_match_numerical_derivatives(self):
        values = gtsam.Values()
        values.insert(0, gtsam.Pose2(0.3, -0.1, 0.2))
        values.insert(1, gtsam.Pose2(0.4, 0.3, 0.5))
        factor = gtsam.CustomFactor(gtsam.noiseModel.Isotropic.Sigma(1, 1.), [0, 1],
                                    lambda *args: gyro_error(0.31, *args))
        jac = [None, None]
        gyro_error(0.31, factor, values, jac)
        for key in (0, 1):
            for axis in range(3):
                delta = np.zeros(3)
                delta[axis] = 1e-6
                plus, minus = gtsam.Values(values), gtsam.Values(values)
                plus.update(key, values.atPose2(key).retract(delta))
                minus.update(key, values.atPose2(key).retract(-delta))
                derivative = (gyro_error(0.31, factor, plus, None) -
                              gyro_error(0.31, factor, minus, None)) / 2e-6
                self.assertAlmostEqual(float(derivative[0]), jac[key][0, axis], places=6)

    def test_gtsam_fuses_gyro_with_lidar_yaw(self):
        graph = IncrementalGraph(Config())
        pose = graph.append(gtsam.Pose2(0.02, 0.01, 0.08), 0.02, 0.2)
        self.assertGreater(pose.theta(), 0.02)
        self.assertLess(pose.theta(), 0.025)
        self.assertTrue(np.linalg.eigvalsh(graph.covariance()).min() > 0)

    def test_stationary_calibration_then_track_actual_sensor_motion(self):
        config = Config(calibration_seconds=0.5)
        estimator = HardwareOdometry(config)
        for t in np.arange(0, 1.01, 0.01):
            estimator.gyro.add(float(t), 0.001)
        room = room_points()
        pose, status = estimator.process(0., room)
        self.assertIsNone(pose)
        self.assertEqual(status['state'], 'calibrating_keep_robot_still')
        pose, _ = estimator.process(0.6, room)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(estimator.bias, 0.001)
        current = transformed(room, gtsam.Pose2(0.03, 0.02, 0).inverse())
        pose, status = estimator.process(0.8, current)
        self.assertEqual(status['state'], 'tracking')
        self.assertAlmostEqual(pose.x(), 0.03, delta=0.003)
        self.assertAlmostEqual(pose.y(), 0.02, delta=0.003)

    def test_motion_during_calibration_requires_restart(self):
        estimator = HardwareOdometry(Config(calibration_seconds=0.5))
        for t in np.arange(0, 1.01, 0.01):
            estimator.gyro.add(float(t), 0.001)
        room = room_points()
        estimator.process(0., room)
        _, status = estimator.process(0.6, transformed(room, gtsam.Pose2(0.1, 0, 0).inverse()))
        self.assertIn('moved_during_calibration', status['state'])
        self.assertIsNone(estimator.graph)

    def test_missing_imu_does_not_silently_become_zero_rotation(self):
        estimator = HardwareOdometry(Config(calibration_seconds=0.5))
        for t in np.arange(0, 0.61, 0.01):
            estimator.gyro.add(float(t), 0.001)
        room = room_points()
        estimator.process(0., room)
        estimator.process(0.6, room)
        pose, status = estimator.process(0.8, room)
        self.assertIsNone(pose)
        self.assertEqual(status['state'], 'waiting_for_imu_coverage')
        self.assertEqual(estimator.graph.count, 1)

    def test_scan_loss_or_clock_reset_does_not_publish_fabricated_pose(self):
        estimator = HardwareOdometry(Config(use_imu=False))
        room = room_points()
        estimator.process(1., room)
        pose, status = estimator.process(2.1, room)
        self.assertIsNone(pose)
        self.assertEqual(status['state'], 'tracking_lost_restart_required')
        self.assertEqual(estimator.graph.count, 1)
        estimator = HardwareOdometry(Config(use_imu=False))
        estimator.process(1., room)
        pose, status = estimator.process(0.5, room)
        self.assertIsNone(pose)
        self.assertIn('time_reset', status['state'])


if __name__ == '__main__':
    unittest.main()
