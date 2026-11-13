# Dataset collection plan (AME 505 proposal, Group 5)

From the proposal (*Project_proposal_Group5*, Oct 20 2026):

* **1,000–2,000 RGB-D images** from **8–10 checkpoints** in campus hallways / labs
* scenes: **normal** and **staged hazards**: *blocked exit*, *obstructed walkway*, *open door*
* conditions: **normal and low light**, **partial occlusion**, **different days (sessions)**
* split **by location + session**, ≈ **70 / 15 / 15** train/val/test, **≥ 1 checkpoint never in training**
* later: 5 live-patrol scenarios × ≥ 5 patrols each (S1 none, S2 single, S3 multiple, S4 low light, S5 occlusion).
  That's evaluation, not this dataset.

## Target: ~1,500 images in 3 sessions (by Wednesday 2026-11-18)

Per checkpoint, per session (≈ 50 images, ~10 min per checkpoint):

| Scene | Light | Setups × frames | Images |
|---|---|---|---|
| normal (nothing staged) | normal | 2 × 6 | 12 |
| normal | low | 1 × 6 | 6 |
| hazard: the checkpoint's main type, 3 different objects/positions | normal | 3 × 6 | 18 |
| hazard | low | 1 × 6 | 6 |
| hazard partially hidden (person/object in front) | normal | 1 × 6 | 6 |
| **total** | | | **~48** |

10 checkpoints × 48 ≈ **480 per session** → 3 sessions ≈ **1,450 images** (≈ 40 % normal / 60 % hazard).
With 8 checkpoints it's ≈ 1,150, still inside the proposal's range.

Assign hazard types so each type has ≥ 3 checkpoints, e.g. 4 exits (blocked exit), 3 corridors/aisles
(obstructed walkway), 3 doors (open door). Use **varied objects**: boxes of different sizes, chairs,
carts, bags, trash can, cables/tripod. Keep the label "hazard" only when the object is **inside the
keep-clear zone** (proposal Fig. 2; the team fixes its size, e.g. door width × ~1 m in front of the
door; OSHA 1910.37 needs exit routes ≥ 28 in / 0.71 m wide); also take
some "object present but OUTSIDE the zone" shots labeled **normal**. These teach the
classifier the difference, and they're the interesting cases for the depth rule.

Sessions: **S1** (Fri/Sat) daytime · **S2** (Sun/Mon) different time of day · **S3** (Tue) evening /
different object set. Low light = ceiling lights off/half, not pitch black.
Hold out one checkpoint (e.g. `cp_10`) from training in all sessions.

## Before collecting (once)

1. Make the 2D map and set the 8–10 checkpoints (`docs/MAPPING_NAVIGATION.md`). Use the
   checkpoint ids (`cp_01`…) as the `--checkpoint` names below.
2. At each checkpoint mark the robot's floor position with tape (so S1–S3 views match), and mark the
   keep-clear zone corners on the floor with tape (for labeling + the depth rule).
3. Permission from the lab/building manager; never block a real fire exit (proposal). Avoid faces.

## Collecting

Start the robot **with the camera**: dashboard → tick **camera** → *Start navigation* (records the
map pose too), or *Start driving*. Drive / navigate to the checkpoint, then on the robot (ssh or a
terminal; `source scripts/env.sh` first):

```bash
# normal scene, 6 frames
ros2 run ame505_bringup capture_dataset --session s1 --checkpoint cp_03 --scene normal --n 6
# hazard: box blocking the exit
ros2 run ame505_bringup capture_dataset --session s1 --checkpoint cp_03 --scene hazard \
    --hazard blocked_exit --objects box --n 6
# low light, occluded
ros2 run ame505_bringup capture_dataset --session s1 --checkpoint cp_03 --scene hazard \
    --hazard blocked_exit --objects chair --light low --occluded 1 --n 6
# object present but outside the keep-clear zone -> normal
ros2 run ame505_bringup capture_dataset --session s1 --checkpoint cp_03 --scene normal \
    --objects box --note "outside zone" --n 6
```

Between bursts, nudge the robot a few cm / a few degrees (Xbox LB) so frames aren't identical.
Each frame = `*_rgb.png` + `*_depth.png` (16-bit, mm, aligned to RGB) under
`data/ssd/dataset/<session>/<checkpoint>/`, and one row in `data/ssd/dataset/index.csv` with all labels +
robot pose. That CSV is the proposal's "staging log" (checkpoint-level ground truth).

Note: the Astra comes through Hiwonder's ROS 1 bridge at **320×240**, a few frames/s. Fine for
YOLO at imgsz 320 and for the depth rule. The tool waits for a NEW frame for each image.

## After each session

```bash
du -sh data/ssd/dataset; tail -3 data/ssd/dataset/index.csv
# copy to the shared Google Drive (images are too big for git):
tar czf ~/dataset_s1.tgz -C /home/ubuntu/share/tmp/ame505_data dataset
```
Then: CVAT labeling (exit door, open door, box, chair, cart, …) → YOLO format; split by
`session` + `checkpoint` columns of `index.csv` (never random).

## Checklist for Wednesday

- [ ] 2D map + 8–10 checkpoints saved, floor marks + keep-clear zones taped
- [ ] S1 ≈ 480 images · S2 ≈ 480 · S3 ≈ 480 (≥ 1,000 minimum)
- [ ] every checkpoint has normal + hazard + low light + occluded
- [ ] index.csv backed up to Drive with the images
- [ ] held-out checkpoint chosen and written down
