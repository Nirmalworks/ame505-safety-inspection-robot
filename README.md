# AME 505 – Autonomous Safety Inspection Robot

Course project for AME 505 (Group 5): a Hiwonder JetAuto (Jetson Nano, mecanum base,
lidar, Astra depth camera) that makes a 2D lidar map (slam_toolbox), localizes on it with
AMCL (lidar + wheel odometry + IMU), patrols inspection checkpoints with Nav2, and checks
them for safety hazards (blocked exit, obstructed walkway, open door).

Platform: Ubuntu 22.04 · ROS 2 Humble · slam_toolbox · AMCL · Nav2 (RTAB-Map 3D optional)

**New here / working offline? Read [CLAUDE.md](CLAUDE.md) and [docs/ROBOT.md](docs/ROBOT.md) first.**

## Folder structure

```
ame505-safety-inspection-robot/
├── src/
│   ├── ame505_description/   robot URDF/xacro + meshes, RViz model view
│   └── ame505_bringup/       drivers, Xbox teleop, RTAB-Map 3D mapping, Nav2, patrol, dashboard
│       └── web/              dashboard + 3D map viewer pages
├── scripts/                  dashboard, env, SSD setup, map export, stop
├── CLAUDE.md                    project context for Claude Code / new teammates
├── docs/ROBOT.md                hardware, software layout, working offline
├── docs/MAPPING_NAVIGATION.md   step-by-step guide
├── docs/LOCALIZATION.md         why localization failed, how it works now, mapping good practice
├── docs/DATA_COLLECTION.md      ML dataset plan (1,000-2,000 RGB-D images, 8-10 checkpoints)
├── debug_logs/                  trimmed logs from robot runs (scripts/save_debug_bundle.sh)
├── docs/GTSAM_SLAM.md           GTSAM lidar/gyro SLAM + safe navigation (lidar + depth + collision monitor)
└── data/ssd -> /home/ubuntu/share/tmp/ame505_data   (SSD, git-ignored)
```

## Quick start

**Easiest:** `~/ame505-safety-inspection-robot/scripts/dashboard.sh` and open http://localhost:8505
(or http://<robot-ip>:8505 from a laptop). Start mapping → drive with Xbox LB (map grows live) →
Stop & Save → pick the map → Start navigation → 🎯 Go to goal.

```bash
source ~/ros2_ws/.robotrc && colcon build --symlink-install && ./scripts/setup_ssd_data.sh  # once
source scripts/env.sh                                                                        # every terminal

ros2 launch ame505_description view_robot.launch.py                       # robot model
ros2 launch ame505_bringup robot.launch.py use_rviz:=true                # drive + see lidar/camera
ros2 launch ame505_bringup mapping_2d.launch.py map:=lab2d              # 2D map (Xbox: hold LB)
./scripts/save_2d_map.sh lab2d                                           # save it (while mapping runs)
ros2 launch ame505_bringup navigation_2d.launch.py map:=lab2d           # AMCL + Nav2 (Y = save checkpoint)
ros2 run ame505_bringup patrol --map lab2d                               # autonomous patrol
ros2 run ame505_bringup capture_dataset --session s1 --checkpoint cp_01 --scene normal --n 6   # ML data
./scripts/save_debug_bundle.sh 6 "note"                                  # logs for offline analysis
ros2 launch ame505_bringup mapping.launch.py map:=lab_map new:=true      # (legacy) 3D RTAB-Map
ros2 launch ame505_bringup navigation.launch.py map:=lab_map             # (legacy) RTAB-Map localization
ros2 launch ame505_bringup rviz.launch.py mode:=robot|mapping|navigation # RViz from a laptop
ros2 launch ame505_bringup gtsam_slam.launch.py                          # GTSAM SLAM (robot still ~5 s)
ros2 run ame505_bringup gtsam_map_check --map lab_map_new                # GTSAM on a saved map
```

Full instructions: [docs/MAPPING_NAVIGATION.md](docs/MAPPING_NAVIGATION.md) · GTSAM: [docs/GTSAM_SLAM.md](docs/GTSAM_SLAM.md)
(once: `./scripts/setup_gtsam.sh`)
