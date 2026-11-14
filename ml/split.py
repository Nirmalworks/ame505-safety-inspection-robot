#!/usr/bin/env python3
"""Split the dataset by LOCATION and SESSION (never by random frame), as the proposal requires.

    python3 ml/split.py <dataset dir> --holdout cp_08          # writes <dataset dir>/splits.csv

* every frame of the --holdout checkpoint goes to test (a place the model never trains on)
* every other (session, checkpoint) group goes whole to train, val or test, aiming at ~70/15/15
Frames of one burst are near-duplicates, so a group must never be shared between splits.
"""
import argparse
import csv
import os
import random


def assign(rows, holdout, seed=505, val=0.15, test=0.15):
    """Return {(session, checkpoint): split}."""
    groups = {}
    for r in rows:
        groups.setdefault((r['session'], r['checkpoint']), []).append(r)
    total = len(rows)
    out = {g: 'test' for g in groups if g[1] in holdout}
    n = {'val': 0, 'test': sum(len(groups[g]) for g in out)}
    rest = sorted(g for g in groups if g not in out)
    random.Random(seed).shuffle(rest)
    for g in rest:
        if n['val'] < val * total:
            out[g] = 'val'
        elif n['test'] < test * total:
            out[g] = 'test'
        else:
            out[g] = 'train'
        if out[g] != 'train':
            n[out[g]] += len(groups[g])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dataset')
    ap.add_argument('--holdout', nargs='+', required=True, help='checkpoint id(s) kept out of training')
    ap.add_argument('--seed', type=int, default=505)
    a = ap.parse_args()
    rows = list(csv.DictReader(open(os.path.join(a.dataset, 'index.csv'))))
    split = assign(rows, set(a.holdout), a.seed)
    for r in rows:
        r['split'] = split[(r['session'], r['checkpoint'])]
    with open(os.path.join(a.dataset, 'splits.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for s in ('train', 'val', 'test'):
        part = [r for r in rows if r['split'] == s]
        haz = sum(r['scene'] == 'hazard' for r in part)
        print(f'{s:5s} {len(part):5d} frames ({100 * len(part) / len(rows):4.1f} %), {haz} hazard, '
              f'checkpoints {sorted({r["checkpoint"] for r in part})}')


if __name__ == '__main__':
    main()
