# Localization & navigation: what was wrong, how it works now, how to map well

## 1. Why the robot didn't know where it was / didn't go to goals (diagnosed 2026-11-13)

Found in `data/ssd/logs/navigation_*.log` (Nov 9 and Nov 12 runs) and live tests:

| # | Problem | Evidence | Fix |
|---|---|---|---|
| 1 | **RTAB-Map localization never re-localized.** It needs camera images to recognise places, but the Astra bridge delivered synced frames at **0.4 Hz** and the `lab_map_new` map had only **11 nodes**. Every attempt was rejected, so the robot pose was just "start at origin + wheel odometry", and it drifted. | `Rejected loop closure 10 -> 42: Not enough inliers 0/12`, `astra_bridge: out: 0.4 Hz`, `WM=11` | **2D lidar localization with AMCL** on a slam_toolbox map (`navigation_2d.launch.py`). The lidar runs at 12 Hz and needs no camera. |
| 2 | **Start pose was wrong by 1.4 m.** RTAB-Map applied the "last localization saved in database" correction. | `Update map correction ... xyz=1.404304` then `Begin navigating from current location (1.40, 0.04)` with the robot really at (0, 0) | AMCL starts at the map's **home** (default = where mapping started); "Set pose" fixes it in one click. |
| 3 | **Goals aborted immediately.** Nav2's `default_server_timeout: 20` ms: on the busy Nano the planner could not even *accept* the request in 20 ms. | `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose` | `default_server_timeout: 1000`, `wait_for_service_timeout: 5000` |
| 4 | **Nav2 reset itself every ~1 min.** The lifecycle manager kills all servers if one misses a 4 s heartbeat (CPU starved). | `CRITICAL FAILURE: SERVER planner_server IS DOWN after not receiving a heartbeat for 4000 ms` | `bond_timeout: 30` s, fewer DWB samples (672 → 300), smoother 20 → 10 Hz |
| 5 | **The Nano is overloaded.** Load average 18–19 on 4 cores, swap ~1.8/2 GB full. VS Code + Pylance + Claude ≈ 1.3 GB. | `uptime`, `free -m` | Don't run VS Code on the robot while navigating. Use a laptop + git. |
| 6 | Depth camera obstacles arrive 5–7 s late. | `collision_monitor: [depth]: ... differ on 7.3 seconds. Ignoring the source.` | Lidar-only by default (`camera` checkbox to opt in). |
| 7 | (earlier) The robot spun on goals: planner failure → Spin/BackUp recoveries; the old 2D grid had phantom obstacles. | see `navigate_no_spin.xml` | No spin/backup recovery, lidar-only ray-traced map. |

## 2. How localization works now (all sensors)

```
wheel encoders (vx, vy, wz) ─┐
IMU (yaw, yaw rate)          ─┴─ EKF (robot_localization) ──> odom -> base_footprint   smooth, drifts slowly
lidar /scan + saved 2D map ──── AMCL particle filter ────────> map  -> odom            absolute, corrects drift
lidar + (optional) depth ────── costmaps + collision monitor   obstacle avoidance
```

* **Wheels + IMU** give smooth motion between lidar updates (the EKF fuses wheel velocity incl.
  sideways and IMU heading; the IMU fixes heading where mecanum wheels slip).
* **Lidar + map (AMCL)** says where on the map: ~1000 particles (= guesses of the pose); each
  lidar scan is compared with the walls in the map and bad guesses die. On the dashboard the
  **light-blue dots** are the particles: a tight cluster = sure, a spread cloud = unsure.
  The dashboard shows `Localization GOOD/FAIR/POOR (± m, ± deg)` from AMCL's covariance.
* **Camera**: not used for localization (too late/slow over the ROS 1 bridge; RTAB-Map proved it).
  It is used for inspection images and optionally for obstacles.

### Using it
1. Put the robot at its **home** (where mapping started, or where you pressed *Set home here*), facing the same way.
2. Dashboard → pick the map → **▶ Start navigation**. Wait for "✅ Ready" (~1 min).
3. Check the red lidar dots lie on the black walls. If not: **📍 Set pose**, click where the robot
   is and drag the direction it faces. The particles snap to the walls within a second or two.
4. **🎯 Go to goal**: click the destination, drag for the final heading.
5. Status says POOR / dots spread? Set pose again. Repeated problems → re-map (section 3).

Command line: `ros2 launch ame505_bringup navigation_2d.launch.py map:=lab2d [initial_x:= initial_y:= initial_yaw:=]`.

### Obstacles
* Global costmap = saved map + live lidar (live data overwrites stale map cells) + inflation 0.35 m.
* Local costmap 3×3 m around the robot, lidar (+ depth if camera on), DWB holonomic controller.
* Collision monitor between controller and motors: **stops** if anything is within ~15 cm of the
  front bumper, slows down so contact is never reached within 1.5 s.
* The lidar only sees the front 180°: the robot never reverses; it strafes/turns instead.
* Goal inside an obstacle / inflated wall → planner fails → status "Could not reach". Click a
  free (white) spot at least ~30 cm from walls.

## 3. Good practice for making a 2D map (slam_toolbox)

1. **Start where the robot will start patrols** (the dock / home spot). That pose becomes (0,0,0).
   Write the spot down or mark it with tape on the floor.
2. Close the dashboard on the robot's own screen and VS Code. Use a laptop browser.
3. Clear the area of people walking around, if possible. Close doors the way they'll be during patrols.
4. Drive **slowly**: hold LB only (≈0.15 m/s), not RB. **Turn slowly**: ≤ 0.5 rad/s, and avoid
   spinning on the spot in open space (few features → scan matching slips).
5. Keep walls in view of the front lidar: drive along corridors, not diagonally across big open rooms.
6. **Close loops**: come back to the start (and to places seen before) from a different
   direction; slam_toolbox then snaps the map together. Finish at the start pose.
7. Watch the live map (dashboard **Map** panel) while driving: walls should stay single and
   sharp. Doubled walls = you went too fast; go back over the area slowly.
8. Map every place a checkpoint will be, with ≥ 0.5 m of free space around it.
9. **Stop & Save**. Then open the map preview: grey = unknown, white = free, black = walls.
   Small grey holes inside rooms are fine. Re-map if walls are doubled or bent.
10. Glass, mirrors and black/shiny surfaces confuse the lidar. Expect gaps there.
11. Re-map when furniture moves a lot. AMCL tolerates some change (people, chairs), not a rearranged lab.

## 4. Debugging next time

```bash
./scripts/save_debug_bundle.sh 6 "what happened"     # on the robot
git add debug_logs && git commit -m "debug: ..." && git push
```
On the laptop look at `debug_logs/<time>/logs/navigation_*.summary.txt` first: the deduplicated
warnings/errors with counts. Things to search for:

| Message | Meaning |
|---|---|
| `Timed out while waiting for action server` | CPU starved / timeouts too short |
| `IS DOWN after not receiving a heartbeat` | Nav2 lifecycle reset (CPU starved) |
| `failed to create plan` / `Failed to make progress` | goal unreachable / blocked / robot pose wrong |
| `Failed to meet update rate` (ekf) | CPU load; harmless if occasional |
| `Message Filter dropping message` (amcl/slam) | TF late: odom/EKF not running or CPU load |
| `system.txt` load average > 8, swap almost full | close VS Code on the robot |
