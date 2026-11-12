# Test GTSAM on the JetAuto

This runs a live, experimental **planar lidar/gyro odometry estimator** against the
JetAuto's ROS 2 sensors. It estimates the robot's path while you drive with the
existing teleop. The same estimator can run on the robot or on a ROS 2 Humble
laptop receiving its topics.

The processing pipeline is:

```text
/scan + sensor TF -> base-frame points -> point-to-line ICP -> relative Pose2
/imu  + sensor TF -> base-frame gyro Z -> stationary bias calibration -> yaw delta
                                      \                  /
                                       GTSAM iSAM2 graph
                                              |
                              /gtsam/odom, /gtsam/path, /gtsam/status
```

This is odometry with gyro yaw fusion, not full mapping/loop-closing SLAM. It does
not integrate accelerometer translation or estimate a changing IMU bias. It does
not read command-derived wheel odometry or SLAM poses as measurements. There are
no drive-command publishers, TF broadcasters, or changes to the coordination stack
in the live node. The existing `slam_toolbox` can continue running for comparison.

## Robot frames and the URDFs

The repo's robot descriptions are **JetAcker** and **MentorPi**, not the HIL
JetAuto. The MentorPi lidar transform differs from the measured JetAuto transform.
Do not load either description to manufacture JetAuto extrinsics.

The existing JetAuto relay already publishes its own `/tf_static`. This estimator
looks up `base_footprint <- <scan.header.frame_id>` and
`base_footprint <- <imu.header.frame_id>` at each sensor timestamp. It uses the
full sensor rotations, so the backwards-mounted lidar and any IMU mounting
rotation are accounted for. No robot meshes are needed for state estimation.

The scan plane must be horizontal within 5 degrees. Sensor mounts must be rigid,
and the robot must operate on a flat floor. Translation in both base X and Y is
estimated, including mecanum sideways motion.

## 1. Start the normal robot sensors

Run in the JetAuto's **ROS 2 Humble container**, not the ROS 1 Melodic host.
Keep an already working relay/SLAM session running. If starting the existing
toolkit from scratch, use its documented startup procedure:

```bash
source ~/jetauto_toolkit/scripts/env.sh
# Only when the relay/SLAM session is not already running:
~/jetauto_toolkit/scripts/relay_slam.sh
```

In another terminal, source the same environment and check:

```bash
source ~/jetauto_toolkit/scripts/env.sh
ros2 topic hz /scan
ros2 topic hz /imu
ros2 topic echo /scan --once --field header
ros2 topic echo /imu --once --field header
ros2 run tf2_ros tf2_echo base_footprint lidar_frame
```

Use the IMU header's actual frame in another `tf2_echo` check. The recorded setup
reports about 12 Hz lidar, 49 Hz IMU, and a lidar x offset of 0.096 m with 180-degree
yaw. Treat live TF as authoritative and investigate discrepancies. Sensor clocks
must agree with ROS time; the existing robot chrony setup is relevant here.

## 2. Install the estimator

Put the updated repo inside the container or use the repo's existing bind mount.
Run from `CSCI-513/gtsam_experiments`:

```bash
source ~/jetauto_toolkit/scripts/env.sh
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip
```

**On Jetson ARM**, use the source build:

```bash
bash setup_hardware.sh source
```

This creates `.venv-ros` with access to the existing ROS installation, installs
the pinned runtime dependencies, and builds upstream GTSAM tag `4.2`. It installs
build tools with `sudo` and keeps GTSAM's source/build files in ignored `.build/`.
The default single compiler job limits peak memory usage; compilation can still
take substantial time and RAM. Allow the build to finish. If it is killed for
memory pressure, build on a machine with sufficient memory or run the estimator
on the ROS 2 laptop receiving the JetAuto's sensor topics.

**On a Linux x86_64 Humble laptop**, use the wheel:

```bash
source /opt/ros/humble/setup.bash
bash setup_hardware.sh wheel
```

Do not use the offline `.venv` to run ROS. Humble's compiled `rclpy` extensions
need its system Python 3.10. Source builds export the private library directory
through `run_hardware.sh`; no system GTSAM installation is required.

If ROS Python modules are missing in a minimal Humble installation:

```bash
sudo apt-get install -y ros-humble-tf2-ros ros-humble-sensor-msgs \
  ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-std-msgs
```

The ARM source build is provided for the hardware environment but has not been
executed on a Jetson in this workspace.

## 3. Optional ROS smoke test before using the robot

Run in a separate ROS domain so the simulated inputs are isolated:

```bash
export LD_LIBRARY_PATH="$PWD/.venv-ros/lib:${LD_LIBRARY_PATH:-}"
ROS_DOMAIN_ID=77 .venv-ros/bin/python ros_smoke_test.py
```

This publishes synthetic laser scans, gyro measurements, and static sensor
transforms, then runs the actual live node for about eight seconds and checks
the estimated path against known motion. It prints `PASS` or raises an error.
Its sensors use `/gtsam_test/*`, `test_base`, and `test_lidar` frames. It does not
access the robot or publish motion commands.

## 4. Run on real sensor data

Keep the robot **stationary** initially, in a room with multiple visible walls
or corners, then run:

```bash
bash run_hardware.sh
```

Wait for `/gtsam/status` to report `tracking`. Initial gyro calibration lasts
at least four seconds and checks the scans for motion. The first accepted pose
defines the origin of `gtsam_odom`. Moving during calibration requires a restart.

In another sourced ROS terminal:

```bash
ros2 topic echo /gtsam/status
ros2 topic echo /gtsam/odom --once
```

In RViz, set **Fixed Frame** to `gtsam_odom` and add a **Path** display on
`/gtsam/path`. No `map -> gtsam_odom` transform is broadcast. For quantitative
comparison with `slam_toolbox`, align the trajectories at a common start pose;
their initial coordinate frames are different.

Use the existing teleop to start slowly: about 0.1 m/s translation and 0.2 rad/s
turning. Try stationary periods, a straight out-and-back, sideways motion, and a
small square returning to a floor mark. Scans are not motion-deskewed, so fast
motion is outside this first test's intended use.

Each run writes ignored `results/hardware/<session>/` files:

- `trajectory.csv`: timestamp, estimated X/Y/yaw, bias, overlap, scan RMSE, processing time.
- `status.jsonl`: tracking/calibration/error reports.
- `parameters.json`: the exact parameters for the run.

`/gtsam/odom` contains a pose estimate. Twist is deliberately unset with large
uncertainty because this backend does not estimate velocity. Pose covariance is
an approximate GTSAM marginal under the configured noise model, not a calibrated
safety bound. Existing coordination code does not consume these outputs.

## 5. Record and replay the hardware test

With the estimator and normal sensor stack running, record in another terminal:

```bash
mkdir -p data
ros2 bag record --qos-profile-overrides-path recording_qos.yaml \
  -o data/jetauto_gtsam_square \
  /scan /imu /tf /tf_static /controller/cmd_vel \
  /gtsam/odom /gtsam/path /gtsam/status
```

Stop recording with Ctrl-C after the final stationary period. Use a fresh output
directory each run. Copy the whole bag directory to the laptop for analysis.
The offline environment can replay the same core with recorded static transforms:

```bash
.venv/bin/python replay_hardware.py data/jetauto_gtsam_square \
  --out results/replay_square
```

For a ROS-level replay, stop the live sensor publishers on the analysis machine,
start the estimator with `bash run_hardware.sh -p use_sim_time:=true`, then play
the bag with `--clock` and only `/scan /imu /tf /tf_static` selected as topics.
Avoid replaying recorded `/gtsam/*` output topics into the new run.

The old `0925` speed bag lacks sensor transforms and a guaranteed stationary
calibration period. A limited **lidar-only** replay using the approximate measured
laser transform is available for checking the frontend:

```bash
.venv/bin/python replay_hardware.py \
  ../jetauto_recordings/speed_test_20261108_0925/bag \
  --lidar-only --lidar-transform 0.096 0 0 0 0 1 0 \
  --out results/replay_0925
```

That run accepted 149 poses and rejected 11 ICP updates in the local check. It
does not validate gyro fusion, the override's calibration, or trajectory accuracy.
Use the new bag's real static TF for a meaningful hardware evaluation.

## Configuration and failure states

Change `jetauto.yaml` or supply a ROS parameter override:

```bash
bash run_hardware.sh -p scan_topic:=/scan_raw
# Explicit lidar-only comparison; IMU fusion is not used in this mode:
bash run_hardware.sh -p use_imu:=false
```

The node never silently substitutes missing IMU data with zero angular velocity.
Poor matches do not advance the trajectory. Single walls or long featureless
corridors can be geometrically ambiguous; the quality checks reject them rather
than treating the estimated translation as measured motion.

| Status | Action |
|---|---|
| `calibrating_keep_robot_still` | Wait without moving the robot. |
| `waiting_for_*_tf` / `waiting_for_lidar_tf_or_valid_scan` | Check header frame IDs and sensor TF. |
| `scan_clock_mismatch_or_stale` | Check source timestamps, clock synchronization, and `/use_sim_time`. |
| `waiting_for_imu_coverage` / `imu_gap` | Check IMU timestamps, rate, and message delivery. |
| `low_scan_overlap`, `high_scan_rmse`, `icp_not_converged` | Slow down and improve overlap/environment features. |
| `degenerate_scan_geometry` | Move the test to a room with nonparallel surfaces/corners. |
| `lidar_gyro_disagreement` | Check sensor frames, clock alignment, and calibration. |
| `*_restart_required` | Stop this estimator and restart from a stationary pose. |
| `scan_stream_stale` / `imu_stream_stale` | Restore the sensor stream; a long gap may require a restart. |

The graph is capped at 3,000 accepted poses (roughly ten minutes at 5 Hz) to bound
session memory. There is no relocalization after a long tracking gap. Noise and
quality thresholds are initial experimental settings, not tuned hardware values.

## What has been verified

Locally: the ROS-free estimator tests cover ray-cast scans through the frontend
and GTSAM backend, mecanum lateral motion, reversed lidar mounting, gyro factor
Jacobians/fusion, stationary bias calibration, missing messages, clock resets,
and rejection of unobservable scans. The original robot recording also runs
through lidar-only replay. Shell/Python syntax and dependencies are checked.

Not verified here: onboard installation/runtime, ARM compilation, real sensor
fusion accuracy, or the ROS smoke test. This workspace has no ROS installation
and its Docker daemon is inaccessible. Run the supplied ROS smoke test and real
recording procedure on the Humble environment before drawing accuracy conclusions.

## Sources

- [GTSAM iSAM2](https://borglab.github.io/gtsam/isam2/)
- [GTSAM 4.2 Python build instructions](https://github.com/borglab/gtsam/blob/4.2/python/README.md)
- [ROS Humble TF buffer API source](https://github.com/ros2/geometry2/blob/humble/tf2_ros_py/tf2_ros/buffer.py)
- Robot-specific evidence: `../jetauto_recordings/robot_setup/README.md`.
