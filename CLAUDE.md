# CLAUDE.md: context for Claude Code (and humans) working on this repo

AME 505 (USC, Fall 2026) Group 5: **Autonomous Safety Inspection Robot**. A Hiwonder JetAuto
patrols 8–10 checkpoints, stops at each one, captures RGB-D images, and flags staged hazards
(blocked exit, obstructed walkway, open door) with a fine-tuned YOLO + depth keep-clear-zone check.
The navigation side (this repo, so far) gets the robot to the checkpoints. Details: [docs/ROBOT.md](docs/ROBOT.md).

## Where code runs

* **Robot**: Jetson Nano 4 GB, Ubuntu 22.04 Docker container, ROS 2 Humble. Repo at
  `~/ame505-safety-inspection-robot`, Hiwonder drivers in `~/ros2_ws` (not in this repo).
  RAM is the bottleneck: VS Code + Claude on the robot use ~1–2 GB and push Nav2 into swap
  (load average 19 seen). **Edit on a laptop, `git push`, then `git pull` + build on the robot.**
* **Laptop (offline)**: you can edit and read logs/maps here, but you can't run the robot stack.
  Use `debug_logs/<time>/` (made by `scripts/save_debug_bundle.sh` on the robot) to analyse runs.

## Build / run (on the robot)

```bash
source scripts/env.sh                         # every terminal (ROS + workspace + device fixes)
colcon build --symlink-install                # after pulling changes (python edits: symlinked)
./scripts/dashboard.sh                        # web UI http://<robot-ip>:8505 (restart: ... restart)
./scripts/stop_all.sh                         # kill every robot process + zero the motors
./scripts/save_debug_bundle.sh 6 "what went wrong"   # logs -> SSD + debug_logs/ for git
```

## Pipeline (2D, default since 2026-11-13)

| Step | Launch | What runs |
|---|---|---|
| drive | `robot.launch.py` | motors/odom (Hiwonder), IMU filter, EKF (wheels + IMU), lidar, Xbox teleop, camera bridge |
| map | `mapping_2d.launch.py map:=NAME` | robot + **slam_toolbox** → `/map`; save with `scripts/save_2d_map.sh NAME` |
| navigate | `navigation_2d.launch.py map:=NAME` | robot + **map_server + AMCL** (lidar) + Nav2 (`nav2_stack.launch.py`) |
| patrol | `ros2 run ame505_bringup patrol --map NAME` | visits `checkpoints/NAME.yaml` |

Legacy 3D: `mapping.launch.py` / `navigation.launch.py` (RTAB-Map, `maps/NAME.db`). Its camera-based
relocalization never worked on this robot (see docs/LOCALIZATION.md); keep it only for 3D scenes.

TF: `map -(AMCL or slam_toolbox)-> odom -(EKF)-> base_footprint -(URDF)-> lidar_frame, depth_cam...`

## Data (not in git): SSD `/home/ubuntu/share/tmp/ame505_data` = `data/ssd`

`maps_2d/NAME.pgm|yaml` · `checkpoints/NAME.yaml` · `home/NAME.yaml` (AMCL start pose) ·
`logs/*.log` (one per dashboard launch) · `debug/` · `maps/*.db` + `maps_3d/` (legacy 3D) · `dataset/` (ML images)

## Hard-won facts (read before changing navigation)

* **Lidar** RPLidar A1, mounted yawed by π (`lidar_frame`). Hiwonder's laser filter keeps only
  the robot-FRONT ~180°; nothing sees behind → no reversing (`min_vel_x: 0`), no BackUp/Spin.
* **Camera** Astra is owned by Hiwonder's ROS 1 container; `astra_bridge.py` republishes it in
  ROS 2 at 320×240, ~3–7 Hz, **1–7 s late**. Too late for moving-obstacle avoidance and useless
  for RTAB-Map relocalization. 2D launches default to `use_camera:=false`.
* **Odometry** EKF (Hiwonder `controller/config/ekf.yaml`): wheel vx, vy, vyaw + IMU yaw/yaw-rate.
  Mecanum wheels slip → AMCL uses `OmniMotionModel` with alphas 0.2–0.25.
* **Nav2 on the Nano**: `default_server_timeout` must be ≫ 20 ms (now 1000) or goals abort at once;
  lifecycle `bond_timeout` 30 s or Nav2 resets itself under load; DWB samples kept at 300.
* "Robot spins on goal" was planner failure → default BT recoveries. We use
  `config/navigate_no_spin.xml` (clear costmaps + wait, no Spin/BackUp).
* Map frame = where mapping started. AMCL starts at the map's **home** (`home/NAME.yaml`, default 0,0,0).
  If the robot starts elsewhere → dashboard "📍 Set pose".
* `pkill -f "ros2 launch"` from a shell whose own command line contains "ros2 launch" kills that
  shell. Use `scripts/stop_all.sh` in its own command.

## Code map

* `src/ame505_bringup/ame505_bringup/dashboard.py`: Flask + rclpy UI server; starts/stops launches,
  live map (`/api/navstate`: map, robot pose, scan, plan, AMCL particles + confidence), goals, checkpoints.
* `src/ame505_bringup/web/index.html`: the dashboard page (vanilla JS, canvas map).
* `config/nav2_params.yaml`: Nav2 + AMCL + collision monitor. `config/slam_toolbox_mapping.yaml`.
* `depth_obstacles.py`: depth image → `/depth_cam/points` for costmaps (only with camera on).
* `gtsam_*`: Shreyash's GTSAM lidar/gyro SLAM experiment ([docs/GTSAM_SLAM.md](docs/GTSAM_SLAM.md)).
* `src/ame505_description`: URDF/meshes (footprint 0.33 × 0.28 m).
* `keep_clear.py` + `inspection.py` + `config/inspection.yaml`: depth-rule hazard check at each patrol
  checkpoint. `ml/`: dataset split, evaluation, baseline, tests ([ml/README.md](ml/README.md)).

Docs: [docs/MAPPING_NAVIGATION.md](docs/MAPPING_NAVIGATION.md) (how to use) ·
[docs/LOCALIZATION.md](docs/LOCALIZATION.md) (why it failed, how it works now, mapping good practice) ·
[docs/DATA_COLLECTION.md](docs/DATA_COLLECTION.md) (ML dataset plan for the proposal).

Conventions: keep changes small, explain Nano-specific numbers in comments, never commit `data/`,
build/, install/, log/. Commit trimmed `debug_logs/` only when useful.
