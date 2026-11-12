#!/usr/bin/env bash
# Run inside the robot's ROS 2 Humble container, or on a Humble laptop.
set -euo pipefail
cd "$(dirname "$0")"
mode="${1:-wheel}"
if [[ "$mode" != wheel && "$mode" != source ]]; then
    echo "Usage: bash setup_hardware.sh [wheel|source]" >&2
    exit 2
fi
if [[ "${ROS_DISTRO:-}" != humble ]]; then
    echo "Source the JetAuto ROS 2 environment (or /opt/ros/humble/setup.bash) first." >&2
    exit 1
fi
if ! /usr/bin/python3 -c 'import rclpy, tf2_ros, sensor_msgs, nav_msgs' ; then
    echo "ROS Python dependencies are missing; see HARDWARE.md." >&2
    exit 1
fi
# ROS binary extensions must use Humble's system interpreter, not Conda/Python 3.12.
/usr/bin/python3 -m venv --system-site-packages .venv-ros
venv_python="$PWD/.venv-ros/bin/python"
"$venv_python" -m pip install -r requirements-hardware.txt
if [[ "$mode" == wheel ]]; then
    if ! "$venv_python" -m pip install --only-binary=:all: 'gtsam==4.2'; then
        echo "No usable GTSAM wheel. On Jetson ARM run: bash setup_hardware.sh source" >&2
        exit 1
    fi
else
    # Explicit source mode installs build tools and compiles the pinned upstream tag.
    sudo apt-get update
    sudo apt-get install -y build-essential cmake git libboost-all-dev python3-dev
    "$venv_python" -m pip install 'pyparsing==3.3.3' 'setuptools<81' wheel
    mkdir -p .build
    if [[ ! -d .build/gtsam-4.2 ]]; then
        git clone --depth 1 --branch 4.2 https://github.com/borglab/gtsam.git .build/gtsam-4.2
    fi
    cmake -S .build/gtsam-4.2 -B .build/gtsam-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$PWD/.venv-ros" \
        -DCMAKE_INSTALL_RPATH="$PWD/.venv-ros/lib" \
        -DGTSAM_BUILD_PYTHON=ON -DGTSAM_PYTHON_VERSION=3.10 \
        -DPYTHON_EXECUTABLE="$venv_python" \
        -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_BUILD_CONVENIENCE_LIBRARIES=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_WITH_TBB=OFF
    # Default one compiler job to limit memory consumption on a Jetson.
    cmake --build .build/gtsam-build --parallel "${GTSAM_BUILD_JOBS:-1}"
    cmake --install .build/gtsam-build
    cmake --build .build/gtsam-build --target python-install --parallel 1
fi
export LD_LIBRARY_PATH="$PWD/.venv-ros/lib:${LD_LIBRARY_PATH:-}"
"$venv_python" -c 'import rclpy, gtsam, scipy; from hardware_core import HardwareOdometry, Config; print("ROS + GTSAM imports OK")'
"$venv_python" -m unittest discover -s . -p 'test_*.py' -v
echo "Setup complete. Keep robot still, then run: bash run_hardware.sh"
