#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ "${ROS_DISTRO:-}" != humble ]]; then
    echo "Source the JetAuto ROS 2 Humble environment first." >&2
    exit 1
fi
if [[ ! -x .venv-ros/bin/python ]]; then
    echo "Run bash setup_hardware.sh first (source mode on Jetson ARM)." >&2
    exit 1
fi
export LD_LIBRARY_PATH="$PWD/.venv-ros/lib:${LD_LIBRARY_PATH:-}"
exec .venv-ros/bin/python live_node.py --ros-args --params-file jetauto.yaml "$@"
