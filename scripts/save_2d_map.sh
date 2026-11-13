#!/usr/bin/env bash
# Export the current /map (from mapping or navigation launch) as .pgm/.yaml for
# reports, the dashboard, or plain Nav2/AMCL.  Usage: save_2d_map.sh [name]
set -euo pipefail
NAME="${1:-lab_map}"
OUT="${AME505_DATA:-/home/ubuntu/share/tmp/ame505_data}/maps_2d"
mkdir -p "$OUT"
ros2 run nav2_map_server map_saver_cli -f "$OUT/$NAME" --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0
ls -lh "$OUT/$NAME".*
