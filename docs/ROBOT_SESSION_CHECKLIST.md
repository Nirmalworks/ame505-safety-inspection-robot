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
| 5 | From a laptop on campus Wi-Fi: `ssh <user>@<robot-ip>`, then `curl -m 3 http://<robot-ip>:8505/api/status` | does SSH work, is the dashboard port open | |
| 6 | From an Ubuntu laptop with ROS 2 Humble, robot running: `ros2 topic list` | does ROS discovery cross campus Wi-Fi (expected: no) | |
| 7 | `ssh -L 8505:localhost:8505 <user>@<robot-ip>`, open `http://localhost:8505` on the laptop | dashboard through an SSH tunnel | |

## 2. Map and checkpoints (the dataset depends on this)

Follow `docs/LOCALIZATION.md` section 3 for mapping practice.

- [ ] Start at the patrol's home spot; mark it with tape. Dashboard: **Start mapping**, name `lab2d`.
- [ ] Drive slowly (LB only), close loops, finish at the start. **Stop & Save**. Check the preview:
      white floor, single sharp walls.
- [ ] **Start navigation** on `lab2d`. Lidar dots on the walls? If not, **Set pose**.
- [ ] Drive to each of the 8–10 inspection spots and press **+ Checkpoint here** (or Xbox `Y`).
      Tape the robot position and the keep-clear zone corners at each one.
- [ ] Measure the narrowest doorway on the route: ______ m. (In simulation a 1.0 m doorway caused
      two Nav2 recoveries.)

## 3. Tests for the fix branch

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

If anything in this table fails: note what happened, run section 4, and go back to `main`
(`git checkout main && colcon build --symlink-install && ./scripts/dashboard.sh restart`).

## 4. Bring it home (5 min)

```bash
./scripts/save_debug_bundle.sh 8 "sunday: map + checkpoints + fix tests"
git add debug_logs docs/ROBOT_SESSION_CHECKLIST.md && git commit -m "debug: sunday session" && git push
```

The bundle now starts with a `== board` section and includes `maps_2d/`, `checkpoints/` and `home/`,
which also gives the simulation a real map and route.

## 5. If there is time left

- [ ] Dataset session S1 (`docs/DATA_COLLECTION.md`): about 48 images per checkpoint.
- [ ] With navigation running and the camera on: `free -m` and 30 s of `tegrastats`. This tells us how
      much memory is left for the detector.
