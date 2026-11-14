# Mapping & Navigation Guide (Dashboard · 2D lidar map · AMCL · Nav2 · Xbox)

Navigation side of the proposal: **make a 2D map → record 8–10 checkpoints → autonomous patrol**.
Since 2026-11-13 the default is **2D**: slam_toolbox mapping + AMCL localization (lidar, wheel
odometry, IMU). Why, and how to map well: [LOCALIZATION.md](LOCALIZATION.md). The old RTAB-Map
3D pipeline is still available (tick "3D map instead" / `mapping.launch.py`).

> **Before you start:** close VS Code on the robot (it uses ~1.3 GB RAM + half a CPU core and makes
> Nav2 miss deadlines). Open the dashboard from a laptop/phone instead.

## The easy way: the dashboard

```bash
~/ame505-safety-inspection-robot/scripts/dashboard.sh
```
Open **http://localhost:8505** on the robot, or **http://\<robot-ip\>:8505** from a laptop on the same Wi-Fi
(the script prints the IP).

| Dashboard | What it does |
|---|---|
| **Live view** | camera, colourised depth (red = near, blue = far), lidar top view |
| **Sensors** | battery, lidar rate + nearest obstacle, camera/depth, odometry, IMU, Xbox buttons, drive commands, explored map area |
| **1 Start mapping** | robot + lidar + slam_toolbox. The 2D map grows **live** in the Map panel with the robot arrow and lidar dots |
| **2 Drive** | hold **LB** on the Xbox controller, slowly |
| **3 Stop & Save** | saves `maps_2d/<name>.pgm/.yaml` |
| **Map panel** | pick a map → ▶ Start navigation → 📍 Set pose if needed → 🎯 Go to goal; light-blue dots = AMCL particles; status shows Localization GOOD/FAIR/POOR |
| **Patrol** | record checkpoints (Y or "+ Checkpoint here"), then Patrol |
| **camera** checkbox | depth camera obstacles + live camera view (needed for dataset capture) |
| **GTSAM SLAM** | start/stop GTSAM next to anything running; orange path on the map ([GTSAM_SLAM.md](GTSAM_SLAM.md)) |
| **E-STOP** | zero velocity + stops patrol/navigation |

Only one thing runs at a time. The dashboard refuses to start if robot software is already
running elsewhere (stop it with `scripts/stop_all.sh`).

---

## Xbox controller (Bluetooth or USB)

* **Bluetooth:** pair once on the robot's **host desktop** (Bluetooth settings). After that,
  switching the controller on is enough. Verified here: *Xbox Series X Controller* over Bluetooth.
* **USB cable:** just plug it in.
* `source scripts/env.sh` (the dashboard does this) fixes the `/dev/input` permissions and the
  SDL button-mapping for Bluetooth.

| Control | Action |
|---|---|
| **Hold LB** | deadman: moves only while held (0.15 m/s, 0.5 rad/s) |
| **Hold RB** | faster (0.30 m/s, 1.0 rad/s) |
| Left stick | forward/back + strafe sideways |
| Right stick | rotate |
| **B** / **START** | E-STOP latch / clear |
| **Y** / **X** | (navigation) save checkpoint / undo |

The dashboard's *Xbox controller* card lights up the buttons you press, so you can check the mapping.

## Hardware notes (this robot)

* **Depth camera:** owned by Hiwonder's ROS 1 container on the host. `robot.launch.py` runs
  `~/jetauto_toolkit/scripts/camera/astra_bridge.py`, which republishes it as
  `/depth_cam/rgb/image_raw` + `/depth_cam/depth/image_raw` (depth aligned to colour, 320×240).
  The ROS 2 Orbbec driver cannot open it ("uvc_open busy").
* **Lidar:** `/dev/lidar` is recreated automatically by `scripts/env.sh`.
* **Memory:** the Nano has 4 GB shared with the GPU. **Close VS Code** on the robot while mapping
  (it uses ~2 GB) and view RViz/dashboard from a laptop when possible.

## Where data is stored (SSD)

```
/home/ubuntu/share/tmp/ame505_data/     (= repo/data/ssd, git-ignored, survives container rebuilds)
├── maps_2d/<map>.pgm/.yaml  2D map used for navigation (AMCL)
├── home/<map>.yaml          AMCL start pose ("Set home here"; default = mapping start)
├── dataset/                 ML images + index.csv (capture_dataset)
├── debug/                   full debug bundles (save_debug_bundle.sh)
├── maps/<map>.db            (legacy 3D) RTAB-Map database
├── maps_3d/<map>_<time>.ply coloured 3D point cloud (MeshLab / CloudCompare / dashboard viewer)
├── checkpoints/<map>.yaml   patrol checkpoints
├── logs/                    launch logs from the dashboard
└── bags/                    rosbags for the dataset
```

---

## Command-line equivalents

Every terminal: `source ~/ame505-safety-inspection-robot/scripts/env.sh`

```bash
# robot model only (no hardware)
ros2 launch ame505_description view_robot.launch.py

# drive + watch lidar / depth camera in RViz (no map)
ros2 launch ame505_bringup robot.launch.py use_rviz:=true

# 3D mapping (no RViz on the robot by default: it exhausts the Nano's RAM; use the laptop command below)
ros2 launch ame505_bringup mapping.launch.py map:=lab_map new:=true
#   leave out new:=true to EXTEND an existing map
#   map_3d:=false     2D-only from lidar (lighter)      use_camera:=false  no camera at all
#   use_rviz:=true    RViz on the robot (depth cloud + camera displays start unticked)
ros2 run ame505_bringup save_3d_map --name lab_map     # while mapping: save .ply
./scripts/save_2d_map.sh lab_map                       # while mapping: save .pgm
# Ctrl+C = stop; RTAB-Map saves data/ssd/maps/lab_map.db on exit

# navigation on the saved map (goals from the dashboard, or RViz "2D Goal Pose" on a laptop; Y on Xbox = checkpoint)
ros2 launch ame505_bringup navigation.launch.py map:=lab_map
ros2 run ame505_bringup patrol --map lab_map --loops 1 --dwell 5

# RViz from a laptop (same Wi-Fi, ROS_DOMAIN_ID=0)
ros2 launch ame505_bringup rviz.launch.py mode:=robot|mapping|navigation
```

## How to drive for a good 3D map

1. Wait ~15 s after *Start mapping* until the *Map* card shows an area.
2. Hold **LB**, drive **slowly** (no RB), turn slowly: the camera only gives ~2–6 frames/s.
3. Keep the camera pointed at what you want in 3D. The lidar builds the 2D map anyway.
4. **Close loops:** drive back to the start and to known places so RTAB-Map corrects drift.
5. **Stop & Save**, then **Open 3D map viewer**.

## Troubleshooting

| Problem | Fix |
|---|---|
| Xbox card says "not seen" | controller on? paired on host? `source scripts/env.sh`, restart dashboard |
| Xbox card says "wrong layout" | something started `joy_node` instead of `game_controller_node`; use our launch files |
| Robot doesn't move | holding **LB**? E-STOP latched (START)? battery > 10.5 V? |
| No camera / depth | Hiwonder's ROS 1 camera container must be running; check `/tmp`-free and the log panel |
| "already running outside the dashboard" | `scripts/stop_all.sh`, then start again |
| No 3D cloud saved | camera wasn't publishing, or `map_3d:=false` |
| Nano freezes | close VS Code, untick RViz, use the dashboard from a laptop |
| Robot spins instead of driving to the goal | fixed: no spin recovery, holonomic controller, live lidar clears stale map obstacles. Old maps: re-map with *new map* |
| Robot shown in the wrong place / goals fail | 📍 Set pose; check red lidar dots match black walls; see [LOCALIZATION.md](LOCALIZATION.md) |
| "Navigation starting" forever, Nav2 resets | Nano overloaded: close VS Code; `./scripts/save_debug_bundle.sh` and look at the summary |
| Map smeared / doubled walls | drive and turn slower, close loops, or `new` map |
