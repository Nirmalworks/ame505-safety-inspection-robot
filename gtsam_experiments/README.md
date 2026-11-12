# GTSAM experiments for JetAuto

**For live JetAuto testing, start with [HARDWARE.md](HARDWARE.md).** It covers
installation on Humble/Jetson, real sensor transforms, live lidar/gyro odometry,
RViz, recording, and replay. The sections below describe the original offline
pose-graph demo and raw data tools.

An isolated starting point for evaluating localization before integrating it with
`coord_core`. This folder can be committed in the existing repo or copied into a
separate repo. It has no imports from the rest of CSCI-513 and requires no ROS
installation for offline analysis. The example bag commands reference recordings from the original CSCI-513 repository;
those recordings and robot setup notes are not included here.

## What works now

- GTSAM 4.2 planar pose-graph optimization from relative motion constraints.
- A reproducible synthetic square demo with a supplied loop closure, CSV outputs,
  an image, and error metrics against synthetic ground truth.
- Offline ROS bag inspection and raw lidar/IMU export using `rosbags`.
- Optional odometry and TF export when those messages are present.
- Live ROS 2 lidar/gyro odometry, JetAuto configuration, and a Jetson source-build path.

The offline `pose_graph.py` demo is a backend and data preparation workflow.
The separate `live_node.py` now implements scan matching, stationary gyro-bias
calibration, and planar gyro-yaw fusion; see the hardware guide. Neither workflow
implements automatic loop detection or full 3D inertial navigation. The demo's
loop closure is deliberately supplied from the synthetic scenario; it is not a
result measured on the robot.

## Setup and first run

From the repository root, on a laptop/desktop:

```bash
cd gtsam_experiments
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-lock.txt
python pose_graph.py --demo --seed 7 --out results/demo
python -m unittest discover -s . -p 'test_*.py' -v
```

Tested on Linux x86_64 with Python 3.10. `requirements.txt` records direct
dependencies; `requirements-lock.txt` records the full verified environment.
NumPy is pinned to 1.26.4 for the GTSAM 4.2 binary wheel. This environment is
separate from the main project's `.venv`. For ROS/Jetson use the separate
`.venv-ros` installation in [HARDWARE.md](HARDWARE.md), including the ARM source
build. That onboard build has not been verified in this workspace.

The demo writes `demo_edges.csv`, `synthetic_truth.csv`, `trajectory.csv`,
`trajectory.png`, and `summary.json` under `results/demo/`. For seed 7, the tested
position RMSE falls from about 0.103 m to 0.019 m using the supplied synthetic loop
constraint. This checks the backend, not hardware accuracy or live performance.

## Inspect and export an existing robot recording

With the environment activated and working directory still `gtsam_experiments/`:

```bash
python bag_tools.py inspect ../jetauto_recordings/speed_test_20261108_0925/bag
python bag_tools.py export ../jetauto_recordings/speed_test_20261108_0925/bag \
  --out results/jetauto_0925
```

The existing `0925` bag contains 1,881 IMU messages and 479 `/scan` messages over
about 38 seconds. It has no TF or odometry messages. The export writes:

| File | Contents |
|---|---|
| `bag_summary.json` | Topic inventory and exported counts |
| `imu.csv` | Angular velocity, acceleration, orientation, sensor covariances |
| `scans.jsonl` | Scan geometry, ranges, intensities, and frame IDs |
| `odometry.csv` | Poses and twists, when recorded; otherwise header only |
| `transforms.csv` | Static/dynamic transforms, when recorded; otherwise header only |

Both bag reception time (`bag_ns`) and sensor header time (`stamp_ns`) are
preserved as integer nanoseconds. Check their alignment before fusion. Missing or
nonfinite laser returns are `null` in JSON, not zero-distance obstacles. Reject
null or out-of-range rays before passing exported scans to any frontend.

Use `--scan-topic /scan_raw` or `--imu-topic <name>` for different topic names.
Only one scan topic is exported to avoid counting duplicate `/scan` and
`/scan_raw` streams as independent sensors. Passing a `.db3` path reads its
whole parent bag directory, so keep `metadata.yaml` and all split files together.

## What your friend should record next

Run the existing relay and `slam_toolbox` setup described in
`../jetauto_recordings/robot_setup/README.md`. In the robot's ROS 2 Humble
container, with this folder available:

```bash
mkdir -p data
ros2 topic list
ros2 bag record --qos-profile-overrides-path recording_qos.yaml \
  -o data/jetauto_square \
  /scan /imu /tf /tf_static /odom /odom_raw /controller/cmd_vel
```

Verify the actual topic names first. The current relay forwards TF but may not
publish standalone `/odom` topics; those are optional for the raw export. Ensure
`/scan`, `/imu`, `/tf`, and `/tf_static` are available. The TF chain should include
`map -> odom -> base_footprint` and the lidar/IMU sensor transforms.

Record a stationary period, a straight run with stops, slow turns, and a square
returning to a marked starting position. Keep recording during the final stationary
period, then stop recording with Ctrl-C. Copy the **whole bag directory** to the
laptop's `gtsam_experiments/data/` and inspect/export it there. Use a fresh bag
output name each time. No part of this experiment sends robot drive commands.

The setup notes report a reversed lidar mount (about 180 degrees yaw) and a gyro
bias around 0.001 rad/s. Use recorded sensor transforms and estimate actual bias;
do not hardcode these as universally correct calibration values. Command-derived
odometry in this repository is not independent evidence of physical motion.

## Feed real relative poses to GTSAM

A future scan matcher should produce this CSV schema:

```csv
i,j,dx,dy,dtheta,sigma_x,sigma_y,sigma_theta,kind
0,1,0.10,0.00,0.01,0.02,0.02,0.02,motion
1,2,0.10,0.00,0.00,0.02,0.02,0.02,motion
```

```bash
python pose_graph.py --edges data/lidar_edges.csv --out results/lidar_graph
```

- IDs start at 0; motion factors form the complete consecutive chain
  `0 -> 1 -> 2 -> ...` with exactly one motion edge per step.
- Each measurement is `T_i.inverse() * T_j`: translation in pose **i's local
  frame**, in meters, and yaw in radians. All graph poses must refer to the same
  physical frame, such as `base_footprint`; convert lidar motion using extrinsics.
- Noise columns are positive one-sigma standard deviations, not variances.
  The diagonal noise model is an initial simplification; use measured noise for
  hardware experiments rather than copying the example values.
- Optional `kind=loop` rows connect existing poses with independently verified
  relative-pose constraints. Loop factors use a Huber loss, which does not make
  an incorrect loop closure safe.
- Pose 0 is anchored to the origin. Output coordinates are local to that origin;
  register them to the experiment's world frame before any controller integration.

A motion chain alone reproduces integrated motion; it cannot remove drift without
additional independent information. Do not fuse SLAM output and its underlying
scan/IMU information as if they were independent. Graph objective reduction measures
constraint consistency; evaluate real accuracy against an external reference.

The hardware implementation checks timestamps/extrinsics and scan quality, and
supports lidar-only or lidar/gyro modes. Compare it against the existing
`slam_toolbox` baseline using measured position
error, stationary heading drift, processing latency, and failure cases. Any safety
integration must address localization uncertainty in addition to the existing
communication-delay margins.

## Commit and push

Code, dependency pins, recording settings, and documentation are tracked. Local
environments, new bags, raw data, plots, and generated outputs are ignored.
The main Python package and ROS workspace do not depend on this experiment.

From the repository root, review and publish when ready:

```bash
git status --short --untracked-files=all
git add README.md gtsam_experiments/
git diff --cached --stat
git diff --cached
git commit -m "Add JetAuto GTSAM hardware test and offline experiments"
git push origin HEAD
```

Use the last command on the branch you intend to publish. Nothing in the setup
automatically commits or pushes. Share large recordings separately or through a
deliberate Git LFS workflow. Keep small reviewed reference measurements in a
separate tracked folder if reproducible hardware benchmarks are added later.

## References

- [GTSAM Pose2 graph example](https://borglab.github.io/gtsam/pose2slamexample/)
- [Pose2 conventions](https://borglab.github.io/gtsam/pose2/)
- [GTSAM IMU factors](https://borglab.github.io/gtsam/imufactor/)
- [rosbags documentation](https://ternaris.gitlab.io/rosbags/)
