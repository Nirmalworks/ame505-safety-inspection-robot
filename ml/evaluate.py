#!/usr/bin/env python3
"""Score hazard / normal predictions against the dataset labels (the proposal's metrics).

    python3 ml/evaluate.py <dataset dir> <predictions.csv> [--split test]

predictions.csv needs the columns file_rgb and pred (1 = hazard, 0 = normal). Reports recall,
precision and F1 for "hazard", the false-alarm rate on normal frames, and recall per hazard
type, light and occlusion. Targets: recall >= 0.90, F1 >= 0.85.
"""
import argparse
import csv
import os


def metrics(pairs):
    """pairs = [(is_hazard, predicted_hazard), ...] -> dict of counts and rates."""
    tp = sum(1 for t, p in pairs if t and p)
    fp = sum(1 for t, p in pairs if not t and p)
    fn = sum(1 for t, p in pairs if t and not p)
    tn = sum(1 for t, p in pairs if not t and not p)
    recall = tp / (tp + fn) if tp + fn else float('nan')
    precision = tp / (tp + fp) if tp + fp else float('nan')
    f1 = 2 * precision * recall / (precision + recall) if tp else 0.0
    return {'n': len(pairs), 'tp': tp, 'fp': fp, 'fn': fn, 'tn': tn, 'recall': recall,
            'precision': precision, 'f1': f1,
            'false_alarm_rate': fp / (fp + tn) if fp + tn else float('nan')}


def report(rows, preds, title=''):
    """rows = dataset rows (dicts), preds = {file_rgb: bool}. Prints and returns the overall metrics."""
    rows = [r for r in rows if r['file_rgb'] in preds]
    pair = lambda rs: [(r['scene'] == 'hazard', preds[r['file_rgb']]) for r in rs]  # noqa: E731
    m = metrics(pair(rows))
    print(f'{title}{m["n"]} frames: recall {m["recall"]:.2f}  precision {m["precision"]:.2f}  F1 {m["f1"]:.2f}  '
          f'false alarms {m["fp"]} of {m["fp"] + m["tn"]} normal frames ({100 * m["false_alarm_rate"]:.0f} %)')
    for col in ('hazard', 'light', 'occluded'):
        for value in sorted({r[col] for r in rows if r['scene'] == 'hazard'}):
            sub = metrics(pair([r for r in rows if r['scene'] == 'hazard' and r[col] == value]))
            print(f'    hazard frames with {col}={value}: recall {sub["recall"]:.2f} ({sub["tp"]}/{sub["tp"] + sub["fn"]})')
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dataset')
    ap.add_argument('predictions')
    ap.add_argument('--split', default='', help='only this split (needs splits.csv from ml/split.py)')
    a = ap.parse_args()
    name = 'splits.csv' if a.split else 'index.csv'
    rows = list(csv.DictReader(open(os.path.join(a.dataset, name))))
    if a.split:
        rows = [r for r in rows if r['split'] == a.split]
    preds = {r['file_rgb']: r['pred'] in ('1', 'True', 'true') for r in csv.DictReader(open(a.predictions))}
    report(rows, preds, f'{a.split or "all"}: ')


if __name__ == '__main__':
    main()
