#!/usr/bin/env python3
"""Run the GTSAM lidar back-end on an EXISTING RTAB-Map map (no robot, no ROS topics needed).

Every RTAB-Map node in <map>.db stores the lidar scan, the wheel/EKF odometry pose and the
loop-closure-optimized map pose. This tool:
  1. matches consecutive scans with the GTSAM point-to-line ICP (gtsam_core.scan_match),
     seeded by wheel odometry; if ICP rejects a pair it keeps the odometry edge (weak),
  2. adds loop closures between any two nodes the robot revisited (ICP-verified, Huber loss),
  3. optimizes the pose graph in the MAP frame (node 1 anchored to RTAB-Map's map pose),
  4. reports the error vs RTAB-Map's map poses and draws everything on the saved 2D map.

    ros2 run ame505_bringup gtsam_map_check --map lab_map_new
-> $AME505_DATA/gtsam/<map>_check.png / .csv / .json

Note: RTAB-Map's map poses also come from lidar ICP on the same scans, so they are a
consistency reference, not independent ground truth.
"""
import argparse
import csv
import json
import math
import os
import sqlite3
import struct
import sys
import zlib

import cv2
import gtsam
import numpy as np

from ame505_bringup.gtsam_core import Config, scan_match
from ame505_bringup.paths import data_dir, map_db

ODOM_SIGMA = np.array([0.10, 0.10, 0.10])      # fallback edge when ICP rejects a pair
LOOP_MAX_DIST, LOOP_MAX_YAW = 1.5, 1.0          # candidate revisits (m, rad)


def _mat(blob):
    """RTAB-Map compressData2 (zlib + rows, cols, cv type) -> (N, channels) array."""
    rows, cols, typ = struct.unpack('<3i', blob[-12:])
    dtype = {0: np.uint8, 1: np.int8, 2: np.uint16, 3: np.int16, 4: np.int32,
             5: np.float32, 6: np.float64}[typ & 7]
    return np.frombuffer(zlib.decompress(blob[:-12]), dtype).reshape(rows * cols, (typ >> 3) + 1)


def _pose2(m34):
    m = np.asarray(m34, dtype=float).reshape(3, 4)
    return gtsam.Pose2(m[0, 3], m[1, 3], math.atan2(m[1, 0], m[0, 0]))


def load_map(db, config):
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    opt = {}
    ids, poses = con.execute('select opt_ids, opt_poses from Admin').fetchone()
    if ids and poses:
        opt = {int(i): _pose2(p) for i, p in zip(_mat(ids).ravel(), _mat(poses).reshape(-1, 12))}
    nodes = []
    for nid, stamp, pose, scan, info in con.execute(
            'select n.id, n.stamp, n.pose, d.scan, d.scan_info from Node n '
            'join Data d on d.id = n.id where d.scan is not null order by n.id'):
        pts = _mat(scan)[:, :2].astype(float)                      # lidar frame x, y
        r = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[np.isfinite(r) & (r >= config.min_range) & (r <= config.max_range)]
        if len(pts) > config.max_points:
            pts = pts[np.linspace(0, len(pts) - 1, config.max_points, dtype=int)]
        lt = np.asarray(struct.unpack('<19f', info[:76])[7:19]).reshape(3, 4)   # lidar -> base
        base = pts @ lt[:2, :2].T + lt[:2, 3]
        nodes.append({'id': nid, 'stamp': stamp, 'odom': _pose2(np.frombuffer(pose, np.float32, 12)),
                      'opt': opt.get(nid), 'pts': base})
    grid = con.execute('select opt_map, opt_map_x_min, opt_map_y_min, opt_map_resolution from Admin').fetchone()
    con.close()
    return nodes, grid


def build_graph(nodes, config):
    graph, initial = gtsam.NonlinearFactorGraph(), gtsam.Values()
    lidar = gtsam.noiseModel.Diagonal.Sigmas(np.array(
        [config.lidar_translation_sigma] * 2 + [config.lidar_yaw_sigma]))
    weak = gtsam.noiseModel.Diagonal.Sigmas(ODOM_SIGMA)
    loop = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.0), lidar)
    origin = nodes[0]['opt'] or nodes[0]['odom']
    graph.add(gtsam.PriorFactorPose2(0, origin, gtsam.noiseModel.Diagonal.Sigmas(np.full(3, 1e-4))))
    initial.insert(0, origin)
    edges = []
    for k in range(1, len(nodes)):
        a, b = nodes[k - 1], nodes[k]
        guess = a['odom'].between(b['odom'])
        try:
            motion, q = scan_match(a['pts'], b['pts'], guess, config)
            graph.add(gtsam.BetweenFactorPose2(k - 1, k, motion, lidar))
            edges.append({'i': k - 1, 'j': k, 'kind': 'icp', **q})
        except ValueError as exc:
            motion = guess
            graph.add(gtsam.BetweenFactorPose2(k - 1, k, motion, weak))
            edges.append({'i': k - 1, 'j': k, 'kind': 'odom_fallback', 'reason': str(exc)})
        initial.insert(k, initial.atPose2(k - 1).compose(motion))
    # Loop closures: non-consecutive nodes that the odometry says are close to each other.
    for i in range(len(nodes)):
        for j in range(i + 2, len(nodes)):
            guess = initial.atPose2(i).between(initial.atPose2(j))
            if math.hypot(guess.x(), guess.y()) > LOOP_MAX_DIST or abs(guess.theta()) > LOOP_MAX_YAW:
                continue
            try:
                rel, q = scan_match(nodes[i]['pts'], nodes[j]['pts'], guess, config)
            except ValueError:
                continue
            graph.add(gtsam.BetweenFactorPose2(i, j, rel, loop))
            edges.append({'i': i, 'j': j, 'kind': 'loop', **q})
    result = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()
    return [result.atPose2(k) for k in range(len(nodes))], [initial.atPose2(k) for k in range(len(nodes))], edges


def rmse(a, b):
    d = [math.hypot(p.x() - q.x(), p.y() - q.y()) for p, q in zip(a, b) if p is not None and q is not None]
    return float(np.sqrt(np.mean(np.square(d)))) if d else None


def draw(path, grid, nodes, est, odom_chain):
    blob, x0, y0, res = grid if grid and grid[0] else (None, 0., 0., 0.05)
    allp = np.vstack([[p.x(), p.y()] for p in est] + [n['opt'] and [n['opt'].x(), n['opt'].y()] or [0, 0] for n in nodes])
    if blob is not None:
        rows, cols = struct.unpack('<2i', blob[-12:-4])
        g = _mat(blob).reshape(rows, cols).astype(np.int16)[::-1]   # row 0 = y_min -> flip
        img = np.full(g.shape + (3,), 128, np.uint8)
        img[g == 0] = 235
        img[g > 0] = 40
    else:
        lo, hi = allp.min(0) - 2, allp.max(0) + 2
        x0, y0 = lo
        img = np.full((int((hi[1] - lo[1]) / res), int((hi[0] - lo[0]) / res), 3), 128, np.uint8)
    scale = max(1, int(round(900 / max(img.shape[:2]))))
    img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    H = img.shape[0]

    def px(x, y):
        return int((x - x0) / res * scale), int(H - 1 - (y - y0) / res * scale)

    for n, p in zip(nodes, est):                         # scans placed at GTSAM poses
        c, s = math.cos(p.theta()), math.sin(p.theta())
        for x, y in n['pts'][::2]:
            cv2.circle(img, px(p.x() + c * x - s * y, p.y() + s * x + c * y), 1, (60, 60, 230), -1)

    def line(poses, color, w):
        pts = [px(p.x(), p.y()) for p in poses if p is not None]
        for a, b in zip(pts, pts[1:]):
            cv2.line(img, a, b, color, w, cv2.LINE_AA)
        for p in pts:
            cv2.circle(img, p, 3 + w, color, -1)
    line(odom_chain, (150, 150, 150), 1)
    line([n['opt'] for n in nodes], (80, 200, 60), 2)
    line(est, (0, 140, 255), 2)
    for txt, col, yy in (('RTAB-Map map poses', (80, 200, 60), 20), ('GTSAM (this tool)', (0, 140, 255), 40),
                         ('wheel odometry', (150, 150, 150), 60), ('lidar at GTSAM poses', (60, 60, 230), 80)):
        cv2.putText(img, txt, (10, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, txt, (10, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    cv2.imwrite(path, img)


def check(map_name):
    db = map_db(map_name)
    if not os.path.exists(db):
        raise SystemExit(f'No saved map "{map_name}" ({db})')
    config = Config()
    nodes, grid = load_map(db, config)
    if len(nodes) < 2:
        raise SystemExit(f'Map "{map_name}" has only {len(nodes)} lidar node(s); drive more while mapping.')
    est, odom_chain, edges = build_graph(nodes, config)
    ref = [n['opt'] for n in nodes]
    # Wheel odometry expressed in the map frame from the same first pose, for comparison.
    o0 = nodes[0]['opt'] or nodes[0]['odom']
    odom_map = [o0.compose(nodes[0]['odom'].between(n['odom'])) for n in nodes]
    out = data_dir('gtsam')
    base = os.path.join(out, f'{map_name}_check')
    with open(base + '.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['node', 'stamp', 'gtsam_x', 'gtsam_y', 'gtsam_yaw', 'rtabmap_x', 'rtabmap_y',
                    'rtabmap_yaw', 'error_m'])
        for n, p in zip(nodes, est):
            r = n['opt']
            w.writerow([n['id'], n['stamp'], p.x(), p.y(), p.theta()] +
                       ([r.x(), r.y(), r.theta(), math.hypot(p.x() - r.x(), p.y() - r.y())] if r else [''] * 4))
    kinds = {k: sum(e['kind'] == k for e in edges) for k in ('icp', 'odom_fallback', 'loop')}
    summary = {
        'map': map_name, 'nodes': len(nodes), 'edges': kinds,
        'gtsam_vs_rtabmap_rmse_m': rmse(est, ref),
        'wheel_odom_vs_rtabmap_rmse_m': rmse(odom_map, ref),
        'max_error_m': max((math.hypot(p.x() - r.x(), p.y() - r.y()) for p, r in zip(est, ref) if r), default=None),
        'edge_details': edges, 'image': base + '.png',
    }
    with open(base + '.json', 'w') as f:
        json.dump(summary, f, indent=2, default=float)
    draw(base + '.png', grid, nodes, est, odom_map)
    return summary


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default='lab_map')
    a = ap.parse_args(argv)
    s = check(a.map)
    fmt = lambda v: '-' if v is None else f'{v:.3f} m'  # noqa: E731
    print(f'[gtsam_map_check] {s["map"]}: {s["nodes"]} nodes | ICP edges {s["edges"]["icp"]}, '
          f'odometry fallbacks {s["edges"]["odom_fallback"]}, loop closures {s["edges"]["loop"]}')
    print(f'  GTSAM vs RTAB-Map map poses: RMSE {fmt(s["gtsam_vs_rtabmap_rmse_m"])}, '
          f'max {fmt(s["max_error_m"])}   (wheel odometry alone: {fmt(s["wheel_odom_vs_rtabmap_rmse_m"])})')
    print(f'  picture: {s["image"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
