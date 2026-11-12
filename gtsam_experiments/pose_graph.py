"""Offline planar GTSAM backend. Inputs are relative poses, not raw lidar scans."""

import argparse
import csv
import json
import math
import os
from pathlib import Path

import gtsam
import numpy as np

FIELDS = ('i', 'j', 'dx', 'dy', 'dtheta', 'sigma_x', 'sigma_y', 'sigma_theta', 'kind')


def read_edges(path):
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        if not set(FIELDS).issubset(reader.fieldnames or []):
            raise ValueError(f'CSV must contain: {", ".join(FIELDS)}')
        edges = []
        for line, row in enumerate(reader, 2):
            try:
                edge = {key: float(row[key]) for key in FIELDS[2:8]}
                edge.update(i=int(row['i']), j=int(row['j']), kind=row['kind'])
                if edge['i'] < 0 or edge['j'] < 0 or edge['i'] == edge['j']:
                    raise ValueError('pose IDs must be distinct nonnegative integers')
                if edge['kind'] not in ('motion', 'loop'):
                    raise ValueError('kind must be motion or loop')
                if not all(math.isfinite(edge[key]) for key in FIELDS[2:8]):
                    raise ValueError('measurements and noise must be finite')
                if any(edge[key] <= 0 for key in FIELDS[5:8]):
                    raise ValueError('noise standard deviations must be positive')
                edges.append(edge)
            except (ValueError, TypeError) as exc:
                raise ValueError(f'{path}:{line}: {exc}') from exc
    if not edges:
        raise ValueError('CSV has no factors')
    return edges


def solve(edges):
    """Anchor pose 0; initialize from the complete 0->1->... motion chain."""
    ids = {e[key] for e in edges for key in ('i', 'j')}
    motion = sorted((e for e in edges if e['kind'] == 'motion'), key=lambda e: e['i'])
    if ids != set(range(len(ids))) or len(motion) != len(ids) - 1:
        raise ValueError('Require contiguous IDs starting at 0 and one motion edge per step')
    if any(e['i'] != k or e['j'] != k + 1 for k, e in enumerate(motion)):
        raise ValueError('Motion edges must form 0->1->2->... without gaps or duplicates')
    initial = gtsam.Values()
    initial.insert(0, gtsam.Pose2())
    for e in motion:
        relative = gtsam.Pose2(e['dx'], e['dy'], e['dtheta'])
        initial.insert(e['j'], initial.atPose2(e['i']).compose(relative))
    graph = gtsam.NonlinearFactorGraph()
    graph.add(gtsam.PriorFactorPose2(
        0, gtsam.Pose2(), gtsam.noiseModel.Diagonal.Sigmas(np.full(3, 1e-6))))
    for e in edges:
        noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([e['sigma_x'], e['sigma_y'], e['sigma_theta']]))
        if e['kind'] == 'loop':
            noise = gtsam.noiseModel.Robust.Create(
                gtsam.noiseModel.mEstimator.Huber.Create(1.345), noise)
        graph.add(gtsam.BetweenFactorPose2(
            e['i'], e['j'], gtsam.Pose2(e['dx'], e['dy'], e['dtheta']), noise))
    result = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()
    summary = {
        'poses': len(ids), 'factors': graph.size(),
        'loop_factors': sum(e['kind'] == 'loop' for e in edges),
        'initial_graph_error': graph.error(initial),
        'optimized_graph_error': graph.error(result),
        'note': 'Graph error measures constraint consistency, not ground-truth accuracy.',
    }
    return initial, result, summary


def save(initial, result, summary, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / 'trajectory.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('id', 'initial_x', 'initial_y', 'initial_yaw', 'x', 'y', 'yaw'))
        for k in range(summary['poses']):
            a, b = initial.atPose2(k), result.atPose2(k)
            writer.writerow((k, a.x(), a.y(), a.theta(), b.x(), b.y(), b.theta()))
    (directory / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    os.environ.setdefault('MPLCONFIGDIR', str(directory.resolve() / '.matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()
    for values, label in ((initial, 'Integrated motion'), (result, 'GTSAM optimized')):
        poses = [values.atPose2(k) for k in range(summary['poses'])]
        ax.plot([p.x() for p in poses], [p.y() for p in poses], '.-', label=label)
    ax.set(xlabel='x [m]', ylabel='y [m]', title='Offline planar pose graph')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True)
    ax.legend()
    fig.savefig(directory / 'trajectory.png', dpi=160, bbox_inches='tight')
    plt.close(fig)


def make_demo(directory, seed):
    """Synthetic square with an explicitly supplied return-to-start constraint."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    truth = [gtsam.Pose2()]
    edges = []
    for k in range(40):
        turn = math.pi / 2 if (k + 1) % 10 == 0 else 0.0
        truth.append(truth[-1].compose(gtsam.Pose2(0.1, 0, turn)))
        dx, dy, theta = np.array([0.105, 0.001, turn + 0.008]) + rng.normal(
            0, [0.008, 0.004, 0.008])
        edges.append((k, k + 1, dx, dy, theta, 0.02, 0.02, 0.02, 'motion'))
    edges.append((0, 40, 0, 0, 0, 0.03, 0.03, 0.03, 'loop'))
    path = directory / 'demo_edges.csv'
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        writer.writerows(edges)
    with (directory / 'synthetic_truth.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('id', 'x', 'y', 'yaw'))
        writer.writerows((k, p.x(), p.y(), p.theta()) for k, p in enumerate(truth))
    return path, truth


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--demo', action='store_true')
    source.add_argument('--edges', type=Path)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    try:
        path, truth = make_demo(args.out, args.seed) if args.demo else (args.edges, None)
        initial, result, summary = solve(read_edges(path))
        if truth is not None:
            summary['synthetic_demo'] = True
            summary['loop_source'] = 'Known synthetic return to start; no loop detection'
            for label, values in (('initial', initial), ('optimized', result)):
                summary[f'{label}_position_rmse_m'] = float(np.sqrt(np.mean([
                    (values.atPose2(k).x() - p.x()) ** 2 +
                    (values.atPose2(k).y() - p.y()) ** 2 for k, p in enumerate(truth)])))
        save(initial, result, summary, args.out)
        print(json.dumps(summary, indent=2))
        print(f'Wrote outputs to {args.out}')
    except (ValueError, OSError) as exc:
        parser.exit(2, f'Error: {exc}\n')


if __name__ == '__main__':
    main()
