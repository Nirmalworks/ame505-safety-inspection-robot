# Robot session checklist (Sunday 2026-11-15)

Goal of the session, in priority order: (1) a good 2D map with 8–10 checkpoints, because the
dataset is due Wed 2026-11-18, (2) confirm the fixes on `fix/estop-stop-state-rviz-defaults`,
(3) bring back facts we could not get from a laptop. Tick the boxes and write results in the
`Result` column; push this file with the debug bundle at the end.

## 0. Before starting (5 min)

- [ ] Battery above 10.5 V. VS Code closed on the robot. Use a laptop browser for the dashboard.
- [ ] Get the branch and build:
  ```bash
  cd ~/ame505-safety-inspection-robot && git fetch && git checkout fix/estop-stop-state-rviz-defaults
  source scripts/env.sh && colcon build --symlink-install && ./scripts/dashboard.sh restart
  ```

## 1. Facts to bring back (10 min)

Run on the Nano's **host** (outside the container) unless it says otherwise.

| # | Command | Why | Result |
|---|---|---|---|
| 1 | `cat /proc/device-tree/model; cat /etc/nv_tegra_release; head -2 /etc/os-release` | which Jetson, JetPack and host Ubuntu | |
| 2 | `docker ps --format '{{.Names}}'` then `docker inspect -f '{{.HostConfig.NetworkMode}} privileged={{.HostConfig.Privileged}}' <name>` | can laptops reach ROS topics (needs `host`) | |
| 3 | `python3 -c "import tensorrt; print(tensorrt.__version__)"` on the host, and again inside the Humble container | where the YOLO detector can run | |
| 4 | Copy `~/jetauto_toolkit/scripts/camera/astra_bridge.py` into `debug_logs/` | to look at the 1–7 s camera delay | |
| 4b | Copy `~/ros2_ws/src/peripherals/config/lidar_filters_config_a1.yaml` into `debug_logs/` (inside the container) | the filter that cuts the lidar to the front 180° | |
| 5 | From a laptop on campus Wi-Fi: `ssh <user>@<robot-ip>`, then `curl -m 3 http://<robot-ip>:8505/api/status` | does SSH work, is the dashboard port open | |
| 6 | From an Ubuntu laptop with ROS 2 Humble, robot running: `ros2 topic list` | does ROS discovery cross campus Wi-Fi (expected: no) | |
| 7 | `ssh -L 8505:localhost:8505 <user>@<robot-ip>`, open `http://localhost:8505` on the laptop | dashboard through an SSH tunnel | |

## 2. Lidar and odometry checks (10 min, robot running: dashboard **Start driving**)

Bags are not committed (`*.db3` is git-ignored): put them on the shared Drive.

- [ ] **Which lidar angles does the robot's own body block?** The model says only the rear ±30°
      (screen, rear shell, camera mount), but the filter drops the whole rear 180°. Put the robot in
      open space, at least 1 m from everything, do not move it, and record the unfiltered scan for 20 s:
      ```bash
      mkdir -p $AME505_DATA/bags && cd $AME505_DATA/bags
      timeout 20 ros2 bag record -o still_scan /scan_raw /scan /tf_static
      ```
- [ ] **Odometry.** Tape a 2.0 m straight line. Note `ros2 topic echo --once /odom --field pose.pose.position`
      before and after driving it (LB only): odom says ______ m for 2.0 m real. Then one full turn on
      the spot back to the same heading: odom yaw changed by ______ deg (should be 360).

## 3. Map and checkpoints (the dataset depends on this)

Follow `docs/LOCALIZATION.md` section 3 for mapping practice.

- [ ] **Record the sensors for the whole mapping drive** (second terminal, start before **Start mapping**,
      Ctrl+C after **Stop & Save**). A few MB per minute; lets us re-run and tune the mapper offline:
      ```bash
      cd $AME505_DATA/bags && ros2 bag record -o mapping_lab2d /scan /scan_raw /odom /odom_raw /imu /tf /tf_static
      ```
- [ ] When coming back to a place already mapped, face the **same way** as the first time: the lidar
      only sees forward, so a loop seen from the opposite direction often does not close.
- [ ] Start at the patrol's home spot; mark it with tape. Dashboard: **Start mapping**, name `lab2d`.
- [ ] Drive slowly (LB only), close loops, finish at the start. **Stop & Save**. Check the preview:
      white floor, single sharp walls.
- [ ] **Start navigation** on `lab2d`. Lidar dots on the walls? If not, **Set pose**.
- [ ] Drive to each of the 8–10 inspection spots and press **+ Checkpoint here** (or Xbox `Y`).
      Tape the robot position and the keep-clear zone corners at each one.
- [ ] Measure the narrowest doorway on the route: ______ m. (In simulation a 1.0 m doorway caused
      two Nav2 recoveries.)

## 4. Tests for the fix branch

Do tests 2, 3 and 6 first **with the wheels off the ground** (robot on a box), then repeat 2 on the
floor in open space. Keep a hand near the power switch.

| # | Do this | Expected | Result |
|---|---|---|---|
| 1 | Navigation running, press **Patrol** | patrol log says `Nav2 is ready for use!` and the robot heads to `cp_01`. Before the fix it could wait forever on `robot_localization/get_state service not available` | |
| 2 | During the patrol press **E-STOP** | wheels stop within about a second and stay stopped; navigation shuts down. Note the time to stop: ____ s | |
| 3 | Start navigation, **Go to goal**, then **E-STOP** mid-drive | same as 2 | |
| 4 | Press **Stop**, then **Stop** again at once | second press answers `Already stopping.` | |
| 5 | Press **Stop**, then **Start navigation** within 2–3 s | refused with `Still stopping the last run...`; works a few seconds later | |
| 6 | Navigation running: `./scripts/dashboard.sh restart` | the old launch is stopped (`pgrep -af "ros2 launch"` shows nothing), wheels still, new dashboard shows idle | |
| 7 | Navigation running: `pkill -9 -f lib/ame505_bringup/dashboard`, then `./scripts/dashboard.sh`, then press **Stop** | answer `Stopped leftover robot software...`; `pgrep -af nav2_container` shows nothing | |
| 8 | `ros2 launch ame505_bringup navigation.launch.py map:=<3D map>` (legacy), only if a 3D map exists | RViz does **not** open on the robot | |

If anything in this table fails: note what happened, run section 5, and go back to `main`
(`git checkout main && colcon build --symlink-install && ./scripts/dashboard.sh restart`).

## 5. Bring it home (5 min)

```bash
./scripts/save_debug_bundle.sh 8 "sunday: map + checkpoints + fix tests"
git add debug_logs docs/ROBOT_SESSION_CHECKLIST.md && git commit -m "debug: sunday session" && git push
```

The bundle now starts with a `== board` section and includes `maps_2d/`, `checkpoints/` and `home/`,
which also gives the simulation a real map and route.

Bags: `cd $AME505_DATA && tar czf ~/bags_sunday.tgz bags` and upload to the shared Drive.

## 6. If there is time left

- [ ] Dataset session S1 (`docs/DATA_COLLECTION.md`): about 48 images per checkpoint.
- [ ] With navigation running and the camera on: `free -m` and 30 s of `tegrastats`. This tells us how
      much memory is left for the detector.
