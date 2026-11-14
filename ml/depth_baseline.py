#!/usr/bin/env python3
"""Run the non-ML keep-clear depth rule over the dataset and score it.

    python3 ml/depth_baseline.py <dataset dir> [--config src/ame505_bringup/config/inspection.yaml]

Writes <dataset dir>/pred_depth_rule.csv (file_rgb, pred, area_m2, ...) and prints the metrics
for every split (after ml/split.py) or for the whole dataset. This is the baseline the trained
models are compared with.
"""
import argparse
import csv
import os
import sys

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'ame505_bringup'))
sys.path.insert(0, HERE)
from ame505_bringup.keep_clear import check_checkpoint, load_config  # noqa: E402
from evaluate import report  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dataset')
    ap.add_argument('--config', default=os.path.join(HERE, '..', 'src', 'ame505_bringup', 'config', 'inspection.yaml'))
    a = ap.parse_args()
    config = load_config(a.config)
    has_splits = os.path.exists(os.path.join(a.dataset, 'splits.csv'))
    rows = list(csv.DictReader(open(os.path.join(a.dataset, 'splits.csv' if has_splits else 'index.csv'))))
    out = []
    for r in rows:
        depth = cv2.imread(os.path.join(a.dataset, r['file_depth']), cv2.IMREAD_UNCHANGED)
        res = check_checkpoint(depth, config, r['checkpoint'])
        out.append({'file_rgb': r['file_rgb'], 'pred': int(res['hazard']), **{k: res[k] for k in ('area_m2', 'points', 'nearest_m', 'valid_pct')}})
    with open(os.path.join(a.dataset, 'pred_depth_rule.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, list(out[0].keys()))
        w.writeheader()
        w.writerows(out)
    preds = {o['file_rgb']: bool(o['pred']) for o in out}
    if has_splits:
        for s in ('train', 'val', 'test'):
            report([r for r in rows if r['split'] == s], preds, f'{s}: ')
    else:
        report(rows, preds, 'all: ')


if __name__ == '__main__':
    main()
