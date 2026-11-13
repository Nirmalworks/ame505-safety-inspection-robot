#!/usr/bin/env python3
"""Stop all 4 wheels by talking to Hiwonder's motor board directly (/dev/rrc).

The STM32 motor board KEEPS THE LAST SPEED it was given. If the ROS driver
(ros_robot_controller) is killed while the robot moves, the wheels keep turning.
Run this after the ROS software has stopped (it needs the serial port free):

    ros2 run ame505_bringup motor_stop
"""
import os
import sys
import time

from ame505_bringup.paths import hiwonder_pkg


def board_stop(device='/dev/rrc', repeats=5, timeout=10):
    """Zero all wheels in a SEPARATE short-lived process (the Hiwonder SDK starts a reader
    thread that never exits and would keep the serial port busy). Returns (ok, message)."""
    import subprocess
    try:
        r = subprocess.run([sys.executable, '-m', 'ame505_bringup.motor_stop', device],
                           capture_output=True, text=True, timeout=timeout)
        out = (r.stdout + r.stderr).strip().splitlines()
        return r.returncode == 0, out[-1] if out else 'motor board: no output'
    except subprocess.TimeoutExpired:
        return False, 'motor board stop timed out'


def _board_stop_inproc(device='/dev/rrc', repeats=5):
    sdk_dir = os.path.join(hiwonder_pkg('ros_robot_controller'), 'ros_robot_controller')
    if sdk_dir not in sys.path:
        sys.path.insert(0, sdk_dir)
    try:
        import ros_robot_controller_sdk as sdk
        board = sdk.Board(device=device, baudrate=1000000, timeout=5)
        for _ in range(repeats):
            board.set_motor_speed([[1, 0.0], [2, 0.0], [3, 0.0], [4, 0.0]])
            time.sleep(0.05)
        try:
            board.port.close()
        except Exception:  # noqa: BLE001
            pass
        return True, 'motor board: all wheels set to 0'
    except Exception as e:  # noqa: BLE001
        return False, f'motor board stop failed: {e}'


def main():
    device = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].startswith('/dev/') else '/dev/rrc'
    ok, msg = _board_stop_inproc(device)
    print(msg, flush=True)
    os._exit(0 if ok else 1)   # don't wait for the SDK's never-ending reader thread


if __name__ == '__main__':
    main()
