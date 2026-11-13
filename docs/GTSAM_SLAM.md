# GTSAM SLAM + safe navigation

Source: Shreyash's `gtsam_experiments/` (branch `origin/gtsam-experiments`, commit `ffc9295`),
integrated into `ame505_bringup`. The estimator core (`gtsam_core.py`) is his
`hardware_core.py`, unchanged. His 12 core tests pass on the Jetson Nano.

## What it is

```text
/scan + sensor TF -> point-to-line ICP ---\
/imu  + sensor TF -> calibrated gyro yaw --+-> GTSAM iSAM2 -> /gtsam/odom  /gtsam/path  /gtsam/status
```

GTSAM is a **read-only** lidar/gyro odometry that runs next to driving, mapping or navigation.
It never sends drive commands and never changes `map`/`odom`. Its frame `gtsam_odom` is attached
to your saved map by a static TF `map -> gtsam_odom`, so the orange GTSAM path is drawn
**on the existing map** in the dashboard and in RViz.

## Install (once per container)

```bash
./scripts/setup_gtsam.sh          # pip --user gtsam 4.3a1 (ARM wheel; 4.2 has none for ARM)
source scripts/env.sh && colcon build --symlink-install --packages-select ame505_bringup
```

## Use it from the dashboard (`./scripts/dashboard.sh` -> http://<robot-ip>:8505)

1. Pick your map and press **▶ Start navigation**, or **Start mapping** / **Start driving**.
2. **GTSAM SLAM** card -> **Start GTSAM**. Keep the robot **still for about 5 s** while the gyro calibrates.
3. Drive. The **orange path** on the map is GTSAM's estimate; the blue arrow is RTAB-Map's.
4. If you started GTSAM before the robot was localized, press **Re-align to map**.
5. **Check on saved map** runs GTSAM over the lidar scans stored in the selected `.db` (no robot needed).

Command line:

```bash
ros2 launch ame505_bringup navigation.launch.py map:=lab_map_new     # terminal 1
ros2 launch ame505_bringup gtsam_slam.launch.py                      # terminal 2 (robot still!)
ros2 launch ame505_bringup rviz.launch.py mode:=navigation           # laptop: orange "GTSAM path"
ros2 run ame505_bringup gtsam_map_check --map lab_map_new            # offline check on a saved map
```

Logs go to `$AME505_DATA/gtsam/<time>_live/`: `trajectory.csv`, `status.jsonl`, `parameters.json`.

## Existing maps

| Map | GTSAM check (vs RTAB-Map map poses) |
|---|---|
| `lab_map_new` (16 nodes) | RMSE **4.8 cm**, max 6.4 cm (wheel odometry alone: 7.5 cm); 6 loop closures |
| `lab_map` (4 nodes, robot did not move) | 0 cm |

RTAB-Map's poses come from the same lidar scans, so this is a consistency check, not ground truth.

## Navigation: "go to the goal without hitting anything"

Why the robot used to spin when you clicked a goal: the planner could not find a path
(`GridBased: failed to create plan`), so Nav2's default recovery spun it 90°, waited, and
backed up. The saved 2D map had 2,617 obstacle cells and **1 free cell**, with obstacles 5 cm
from the start pose. Late-stamped depth-camera frames had been placed at the wrong poses.

What changed:

| | Before | Now |
|---|---|---|
| Controller | Hiwonder DWB: no sideways motion (`max_vel_y 0`), so it rotated in place to turn | Holonomic DWB (mecanum strafing), max 0.20 m/s, real footprint |
| Recoveries | Spin 90° + back up blind | Clear costmaps + wait, then replan (`config/navigate_no_spin.xml`) |
| Sensors | Lidar only, 0.1 m radius in the global costmap | Saved map **plus live lidar plus live depth camera** in both costmaps |
| Stale map obstacles | Permanent | Live lidar **overwrites** the saved map wherever it can see (`combination_method: 0`) |
| Last-line safety | None | `collision_monitor` on raw lidar and depth: slows before contact, stops if anything is within ~15 cm in front |
| New maps' 2D grid | Lidar + late depth frames | Lidar ray-traced (free floor + walls); depth stays live in Nav2 |

Old maps still work for localization and for GTSAM. For the cleanest navigation, re-map once
with **new map** ticked (the grid now records free floor).

## Limits (honest)

* GTSAM is odometry with gyro fusion and map anchoring. It does no loop closing live, and
  nothing in navigation consumes its output (it is for comparison and monitoring).
* The A1 lidar filter keeps only about 180° and the camera sees the front, so the robot never reverses.
* Depth frames arrive about 1–2 s late; they are skipped while the robot turns fast (> 0.35 rad/s).
* Noise and quality thresholds are initial settings, not calibrated safety bounds.
