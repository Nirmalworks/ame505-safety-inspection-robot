"""Check graph correction and reject inputs that cannot define a valid chain."""

import csv
from pathlib import Path
import tempfile
import unittest

import numpy as np

from pose_graph import FIELDS, make_demo, read_edges, solve


class PoseGraphTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path, self.truth = make_demo(self.directory.name, seed=7)
        self.edges = read_edges(self.path)

    def test_loop_reduces_error_against_synthetic_truth(self):
        initial, optimized, summary = solve(self.edges)
        def rmse(values):
            return np.sqrt(np.mean([
                (values.atPose2(k).x() - p.x()) ** 2 +
                (values.atPose2(k).y() - p.y()) ** 2
                for k, p in enumerate(self.truth)]))
        self.assertLess(rmse(optimized), rmse(initial))
        self.assertLess(summary['optimized_graph_error'], summary['initial_graph_error'])

    def test_motion_only_has_no_independent_drift_correction(self):
        initial, optimized, summary = solve([e for e in self.edges if e['kind'] == 'motion'])
        self.assertEqual(summary['loop_factors'], 0)
        np.testing.assert_allclose(optimized.atPose2(40).matrix(),
                                   initial.atPose2(40).matrix(), atol=1e-8)

    def test_missing_motion_edge_is_rejected(self):
        with self.assertRaises(ValueError):
            solve(self.edges[1:])

    def test_duplicate_motion_edge_is_rejected(self):
        with self.assertRaises(ValueError):
            solve(self.edges + [self.edges[0]])

    def test_invalid_noise_is_rejected_by_csv_reader(self):
        path = Path(self.directory.name) / 'bad.csv'
        for invalid in ('0', '-1', 'nan', 'inf'):
            with self.subTest(noise=invalid):
                with path.open('w', newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(FIELDS)
                    writer.writerow((0, 1, 0.1, 0, 0, invalid, 0.02, 0.02, 'motion'))
                with self.assertRaises(ValueError):
                    read_edges(path)


if __name__ == '__main__':
    unittest.main()
