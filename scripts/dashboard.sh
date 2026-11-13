#!/usr/bin/env bash
# Start the AME 505 dashboard: http://localhost:8505  (laptop: http://<robot-ip>:8505)
#   ./dashboard.sh          start (or tell you it's already running)
#   ./dashboard.sh restart  stop the running dashboard and start a fresh one
source "$(dirname "$0")/env.sh"
PORT="${AME505_DASHBOARD_PORT:-8505}"
IPS="$(hostname -I 2>/dev/null)"

running_pid() { pgrep -f "lib/ame505_bringup/dashboard" | head -1; }

if [ "$1" = restart ] && [ -n "$(running_pid)" ]; then
  echo "Stopping old dashboard (pid $(running_pid))..."
  pkill -INT -f "lib/ame505_bringup/dashboard"
  for i in $(seq 15); do [ -z "$(running_pid)" ] && break; sleep 1; done
  # rclpy/Flask sometimes ignore Ctrl+C: escalate so the port is really free
  # (on TERM the dashboard first stops its running launch, which can take ~40 s on the Nano)
  [ -n "$(running_pid)" ] && pkill -TERM -f "lib/ame505_bringup/dashboard"
  for i in $(seq 45); do [ -z "$(running_pid)" ] && break; sleep 1; done
  [ -n "$(running_pid)" ] && pkill -KILL -f "lib/ame505_bringup/dashboard" && sleep 1
  ros2 run ame505_bringup motor_stop >/dev/null 2>&1 || true
fi

if curl -s -m 2 "localhost:$PORT/api/status" >/dev/null 2>&1; then
  echo "Dashboard is ALREADY RUNNING -> open http://localhost:$PORT"
  for ip in $IPS; do echo "                        laptop: http://$ip:$PORT"; done
  echo "(to restart it: $0 restart)"
  exit 0
fi

ame505_deprioritize_editor
echo "Starting dashboard -> http://localhost:$PORT"
for ip in $IPS; do echo "                laptop: http://$ip:$PORT"; done
exec ros2 run ame505_bringup dashboard
