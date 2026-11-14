# Hazard detection: dataset tools and the depth-rule baseline

Runs on a laptop with Python 3, NumPy, OpenCV and PyYAML. No ROS, no robot.

| Step | Command | What it does |
|---|---|---|
| Split | `python3 ml/split.py <dataset> --holdout cp_10` | writes `splits.csv`: whole (session, checkpoint) groups go to train / val / test (~70/15/15); the hold-out checkpoint is only ever in test |
| Baseline | `python3 ml/depth_baseline.py <dataset>` | runs the keep-clear depth rule on every frame, writes `pred_depth_rule.csv`, prints recall / precision / F1 / false alarms per split |
| Score a model | `python3 ml/evaluate.py <dataset> <predictions.csv> --split test` | same metrics for any predictions file with columns `file_rgb`, `pred` (1 = hazard) |
| Tests | `python3 -m pytest ml/test_ml.py` | geometry of the rule, split rules, metrics |
| Placeholder data | `python3 ml/fake_dataset.py /tmp/fake` | rendered floor / wall / box scenes in the real file layout, to try the commands above |

`<dataset>` is the folder with `index.csv` that `capture_dataset` writes (`$AME505_DATA/dataset`).

## The depth rule

`src/ame505_bringup/ame505_bringup/keep_clear.py`: every depth pixel becomes a 3D point in the robot
frame; the frontal area of the points inside the keep-clear zone and above the floor is added up;
at least `min_area_m2` (default 0.01 m², a 10 x 10 cm object) means hazard. Zones and the threshold
are in `src/ame505_bringup/config/inspection.yaml`, with per-checkpoint overrides.

Limits:
* It needs the robot at the taped checkpoint position, since the zone is fixed in the robot frame.
* It cannot detect an **open door**: nothing is inside the zone. That hazard needs the trained model.
* Camera numbers are assumed (Astra at 320x240: fx = fy = 285; mount 0.207 m high, level, from the
  URDF). A camera tilted by a few degrees makes the floor count as an object: set `camera.pitch_deg`.
  On the robot the node takes the intrinsics from `/depth_cam/rgb/camera_info`.
* So far it has only been run on rendered placeholder scenes. Its real accuracy is unknown.

## On the robot

`ros2 run ame505_bringup inspection` next to navigation (camera on) and the patrol: at each
checkpoint it waits 3 s, takes a new RGB-D frame, runs the rule and writes
`$AME505_DATA/inspections/index.csv` plus the image pair. Results are also published on
`inspection/result`.

## Not here yet

YOLO training and TensorRT export (need the CVAT labels), the checkpoint classifier comparison,
the detector inside the inspection node, and inspection results on the dashboard.
