#!/usr/bin/env bash
# Create the SSD data folders and link them into the repo as ./data (git-ignored).
set -euo pipefail
DATA="${AME505_DATA:-/home/ubuntu/share/tmp/ame505_data}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$DATA"/{maps,checkpoints,bags,maps_2d,maps_3d}
mkdir -p "$ROOT/data"; ln -sfn "$DATA" "$ROOT/data/ssd"
echo "Data dir: $DATA  (linked as $ROOT/data/ssd)"
df -h "$DATA" | tail -1
