#!/usr/bin/env bash
# Install GTSAM (Python) for the gtsam_slam / gtsam_map_check nodes. Run once per container.
#
# PyPI has no gtsam==4.2 wheel for ARM (Jetson); 4.3a1 ships a cp310 aarch64 wheel, so no
# multi-hour source build is needed. Shreyash's core + 17 tests pass with it on this Nano.
# --user: no sudo, no venv; --no-deps: keeps the system NumPy / SciPy that ROS uses.
set -euo pipefail
python3 -m pip install --user --no-deps 'gtsam==4.3a1'
cd /
python3 -W ignore -c 'import gtsam, scipy.spatial; print("GTSAM OK:", gtsam.__file__)'
