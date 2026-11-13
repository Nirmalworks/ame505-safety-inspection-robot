# The robot and the project

## Project (AME 505 Machine Learning for Engineering Applications, USC Fall 2026, Group 5)

**Autonomous Safety Inspection Robot.** Team: Leo Crampe, Nirmal Kumar Jagannathan, Soowhan Yi,
Ryan O'Gorman (GTSAM experiments: Shreyash).

The robot patrols a fixed route of 8–10 checkpoints in campus hallways/labs, stops at each one,
captures an aligned RGB-D image and decides *normal* vs *hazard* (blocked exit, obstructed
walkway, open door). ML: fine-tuned YOLO (from COCO) + a checkpoint classifier (kNN / decision
tree / SVM / CNN) compared against a non-ML depth rule (points above the floor inside the
keep-clear zone). Targets: hazard recall ≥ 0.90, F1 ≥ 0.85, ≤ 1 false alarm per patrol,
≤ 200 ms inference on the Jetson, ≥ 80 % patrols without intervention. Results go on a GUI dashboard.

Workflow: **map + checkpoints → data collection → labeling (CVAT) → training → TensorRT on the
Jetson → live patrol → hazard decision → dashboard.** Status (2026-11-13): mapping, localization,
navigation, checkpoints, patrol, dashboard done; dataset collection next ([DATA_COLLECTION.md](DATA_COLLECTION.md)).

## Hardware: Hiwonder JetAuto

| Part | Details | ROS 2 |
|---|---|---|
| Computer | NVIDIA Jetson Nano 4 GB (RAM shared with GPU), SSD bind-mounted at `/home/ubuntu/share` | Docker container, Ubuntu 22.04, ROS 2 Humble |
| Base | mecanum wheels (moves sideways), STM32 board: motors, IMU, battery, buzzer | `ros_robot_controller`, `/controller/cmd_vel`, `/odom_raw`, `/imu`, `/ros_robot_controller/battery` |
| Footprint | 0.33 m × 0.28 m (`base_footprint`) | `src/ame505_description` URDF |
| Lidar | Slamtec RPLidar A1, 12 Hz, `/dev/lidar` (CH340 on hub port 4) | `/scan_raw` → laser filter → `/scan`; front ~180° only (mount yawed π) |
| Depth camera | Orbbec Astra, owned by Hiwonder's **ROS 1** container | `astra_bridge.py` → `/depth_cam/rgb/image_raw`, `/depth_cam/depth/image_raw` (320×240, depth aligned, 1–7 s latency) |
| Odometry | wheel encoders + IMU fused by `robot_localization` EKF | `/odom`, TF `odom → base_footprint` |
| Controller | Xbox Series X pad (Bluetooth or USB) | `/joy` → `xbox_teleop` (hold LB = deadman) |
| Arm / screen | present in the URDF (`urdf/pro`), not used | — |

Battery: charge above ~10.5 V; low battery = weak motors and odometry errors.

## Software layout

```
~/ros2_ws                       Hiwonder drivers (robot_controller, odom_publisher, peripherals: lidar/IMU)
~/jetauto_toolkit               camera bridge, Hiwonder tools (not in this repo)
~/ame505-safety-inspection-robot   THIS repo (ROS 2 workspace)
  src/ame505_description        URDF + meshes
  src/ame505_bringup            launch files, configs, dashboard, tools (see CLAUDE.md "Code map")
  scripts/                      env.sh, dashboard.sh, stop_all.sh, save_2d_map.sh, save_debug_bundle.sh, ...
  docs/                         this folder
  debug_logs/                   trimmed logs committed for offline analysis
/home/ubuntu/share/tmp/ame505_data   (= data/ssd) maps, checkpoints, logs, dataset (NOT in git)
```

## Working offline on a laptop

```bash
git clone https://github.com/Nirmalworks/ame505-safety-inspection-robot.git
# read CLAUDE.md + docs/, analyse debug_logs/<time>/, edit code, then:
git commit -am "..." && git push
# on the robot:
cd ~/ame505-safety-inspection-robot && git pull && source scripts/env.sh && colcon build --symlink-install
./scripts/dashboard.sh restart
```
A laptop without ROS can still edit Python/launch/YAML/HTML and read logs. To view the robot live
from the laptop just open `http://<robot-ip>:8505` (same Wi-Fi). No VS Code on the robot needed.
