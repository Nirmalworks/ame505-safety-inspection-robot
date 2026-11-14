#!/usr/bin/env python3
"""Make a PLACEHOLDER dataset in the same layout capture_dataset writes, to test the pipeline
before real images exist. Depth is rendered from simple geometry (floor, back wall, one box);
RGB is a grey picture of the depth. Never mix this with real data.

    python3 ml/fake_dataset.py /tmp/fake_dataset
"""
import csv
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'ame505_bringup'))
from ame505_bringup.keep_clear import DEFAULT_CAMERA  # noqa: E402

# same columns as capture_dataset.FIELDS (not imported: that module needs ROS)
FIELDS = ['file_rgb', 'file_depth', 'stamp', 'session', 'checkpoint', 'scene', 'hazard', 'objects',
          'light', 'occluded', 'note', 'pose_frame', 'x', 'y', 'yaw', 'depth_valid_pct', 'rgb_age_s']
W, H = 320, 240


def render(box=None, wall_x=3.0, cam=DEFAULT_CAMERA, rng=None):
    """Depth image (uint16 mm) of a floor, a wall at wall_x and an optional box.
    box = (x_centre, y_centre, size_x, size_y, height) in the robot frame, standing on the floor."""
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    dy = -(u - cam['cx']) / cam['fx']                 # ray = (1, dy, dz) per metre of depth
    dz = -(v - cam['cy']) / cam['fy']
    depth = np.full((H, W), wall_x - cam['x'], np.float32)
    with np.errstate(divide='ignore', invalid='ignore'):
        floor = np.where(dz < 0, cam['height'] / -dz, np.inf)
        depth = np.minimum(depth, floor)
        if box:
            bx, by, sx, sy, bh = box
            lo = np.array([bx - sx / 2 - cam['x'], by - sy / 2, -cam['height']])
            hi = np.array([bx + sx / 2 - cam['x'], by + sy / 2, bh - cam['height']])
            d = np.stack([np.ones_like(dy), dy, dz], -1)
            t1, t2 = lo / d, hi / d
            near, far = np.minimum(t1, t2).max(-1), np.maximum(t1, t2).min(-1)
            depth = np.where((near <= far) & (near > 0), np.minimum(depth, near), depth)
    if rng is not None:
        depth = depth * (1 + rng.normal(0, 0.005, depth.shape))      # 0.5 % range noise
        depth[rng.random(depth.shape) < 0.03] = 0                    # 3 % missing pixels
    return (np.clip(depth, 0, 8) * 1000).astype(np.uint16)


def main(out):
    rng = np.random.default_rng(505)
    rows = []
    for session in ('s1', 's2', 's3'):
        for c in range(1, 9):
            cp, hazard = f'cp_{c:02d}', ('blocked_exit', 'obstructed_walkway')[c % 2]
            for scene, n in (('normal', 6), ('normal_outside', 3), ('hazard', 9)):
                for k in range(n):
                    if scene == 'hazard':                            # box inside the keep-clear zone
                        box = (rng.uniform(0.7, 1.3), rng.uniform(-0.25, 0.25), 0.3, 0.3, rng.uniform(0.2, 0.5))
                    elif scene == 'normal_outside':                  # box present, but beside the zone
                        box = (rng.uniform(0.8, 1.4), rng.choice([-1, 1]) * rng.uniform(0.75, 1.0), 0.3, 0.3, 0.4)
                    else:
                        box = None
                    depth = render(box, wall_x=rng.uniform(2.5, 4.0), rng=rng)
                    d = os.path.join(out, session, cp)
                    os.makedirs(d, exist_ok=True)
                    base = os.path.join(d, f'{scene}_{k:02d}')
                    cv2.imwrite(base + '_depth.png', depth)
                    cv2.imwrite(base + '_rgb.png', cv2.cvtColor((np.clip(depth / 16, 0, 255)).astype(np.uint8), cv2.COLOR_GRAY2BGR))
                    is_hazard = scene == 'hazard'
                    rows.append({'file_rgb': os.path.relpath(base + '_rgb.png', out),
                                 'file_depth': os.path.relpath(base + '_depth.png', out),
                                 'stamp': '0', 'session': session, 'checkpoint': cp,
                                 'scene': 'hazard' if is_hazard else 'normal',
                                 'hazard': hazard if is_hazard else 'none',
                                 'objects': 'box' if box else '', 'light': 'normal', 'occluded': '0',
                                 'note': 'FAKE' + (' outside zone' if scene == 'normal_outside' else ''),
                                 'pose_frame': 'map', 'x': 0, 'y': 0, 'yaw': 0,
                                 'depth_valid_pct': round(100.0 * float((depth > 0).mean()), 1), 'rgb_age_s': 0})
    with open(os.path.join(out, 'index.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f'wrote {len(rows)} FAKE frames to {out}')


if __name__ == '__main__':
    main(sys.argv[1])
