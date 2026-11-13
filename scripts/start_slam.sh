#!/usr/bin/env bash
# ONE command: dashboard + navigation on a saved map + GTSAM SLAM.
#
#   ~/ame505-safety-inspection-robot/scripts/start_slam.sh              # map lab_map_new
#   ~/ame505-safety-inspection-robot/scripts/start_slam.sh lab_map      # another map
#
# Put the robot where mapping started and DON'T touch it until "GTSAM started" is printed
# (gyro calibration). Then open the printed URL (laptop/phone on the same Wi-Fi).
# Everything keeps running after this terminal is closed. Stop all: scripts/stop_all.sh
MAP="${1:-lab_map_new}"
cd "$(dirname "$0")/.." || exit 1
source scripts/env.sh >/dev/null 2>&1
PORT="${AME505_DASHBOARD_PORT:-8505}"
API="http://localhost:$PORT/api"
LOG="$AME505_DATA/logs/dashboard.log"

say() { printf '\n\033[1;36m[ame505] %s\033[0m\n' "$*"; }
post() { curl -s -m 15 -X POST -H 'Content-Type: application/json' -d "$2" "$API/$1"; echo; }
rate() { curl -s -m 3 "$API/status" | python3 -c "import sys,json; print(json.load(sys.stdin)['rates'].get('$1',0))" 2>/dev/null || echo 0; }

# 1. one-time setup (skipped when already done)
python3 -c 'import gtsam' 2>/dev/null || { say "Installing GTSAM..."; ./scripts/setup_gtsam.sh || exit 1; }
if [ ! -e install/ame505_bringup/lib/ame505_bringup/gtsam_slam ]; then
  say "Building workspace..."; colcon build --symlink-install --packages-select ame505_bringup || exit 1
  source scripts/env.sh >/dev/null 2>&1
fi

ame505_deprioritize_editor
# 2. dashboard (background, survives closing the terminal)
if ! curl -s -m 2 "$API/status" >/dev/null 2>&1; then
  say "Starting dashboard (log: $LOG)"
  nohup ros2 run ame505_bringup dashboard >"$LOG" 2>&1 </dev/null &
  for i in $(seq 40); do curl -s -m 2 "$API/status" >/dev/null 2>&1 && break; sleep 1; done
  curl -s -m 2 "$API/status" >/dev/null 2>&1 || { echo "Dashboard did not start - see $LOG"; tail -20 "$LOG"; exit 1; }
fi

# 3. navigation on the saved map (lidar + depth + odometry, Nav2 with collision monitor)
say "Starting navigation on map '$MAP' - robot must be where mapping started, keep it STILL"
post start_navigation "{\"map\": \"$MAP\"}"

# 4. wait for lidar + IMU, then GTSAM
say "Waiting for lidar and IMU..."
for i in $(seq 90); do
  s=$(rate /scan); m=$(rate /imu)
  python3 -c "import sys; sys.exit(0 if float('$s')>1 and float('$m')>5 else 1)" && break
  sleep 1
done
sleep 3
say "Starting GTSAM - keep the robot STILL for 5 more seconds"
post start_gtsam '{"use_imu": true}'
sleep 6

say "READY. Open the dashboard:"
echo "   http://localhost:$PORT"
for ip in $(hostname -I 2>/dev/null); do echo "   http://$ip:$PORT"; done
cat <<EOF

  * Orange path = GTSAM SLAM on your map, blue arrow = RTAB-Map.
  * Navigation needs ~1-2 min on the Nano. Then: 🎯 Go to goal -> click on the map.
  * If the orange path is offset: GTSAM card -> "Re-align to map".
  * Stop everything:  ~/ame505-safety-inspection-robot/scripts/stop_all.sh
EOF
# open the dashboard in a browser on the robot's screen (if there is one)
if [ -n "${DISPLAY:-}" ] && command -v xdg-open >/dev/null; then
  nohup xdg-open "http://localhost:$PORT" >/dev/null 2>&1 &
fi
