#!/usr/bin/env python3
"""Record patrol checkpoints with the Xbox controller.

While the robot is localized on a map (navigation.launch.py), drive it to an
inspection spot, face it toward what the camera must see, and press:
    Y -> save current pose (map -> base_footprint) as the next checkpoint
    X -> delete the last checkpoint
Checkpoints are written immediately to <AME505_DATA>/checkpoints/<map>.yaml.
"""
import math
import os

import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformListener

from ame505_bringup.paths import checkpoints_file

try:
    from ros_robot_controller_msgs.msg import BuzzerState
except ImportError:  # buzzer feedback is optional
    BuzzerState = None


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CheckpointRecorder(Node):
    def __init__(self):
        super().__init__('checkpoint_recorder')
        map_name = self.declare_parameter('map', 'lab_map').value
        self.file = self.declare_parameter('file', '').value or checkpoints_file(map_name)
        self.btn_save = self.declare_parameter('button_save', 3).value   # Y
        self.btn_undo = self.declare_parameter('button_undo', 2).value   # X
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.buzzer = (self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
                       if BuzzerState else None)
        self.prev = {}
        self.checkpoints = self.load()
        self.get_logger().info(
            f'Checkpoint file: {self.file} ({len(self.checkpoints)} loaded). '
            'Y = save pose, X = undo last.')

    def load(self):
        if os.path.exists(self.file):
            with open(self.file) as f:
                return (yaml.safe_load(f) or {}).get('checkpoints', [])
        return []

    def save(self):
        os.makedirs(os.path.dirname(self.file), exist_ok=True)
        with open(self.file, 'w') as f:
            yaml.safe_dump({'frame_id': self.map_frame, 'checkpoints': self.checkpoints},
                           f, sort_keys=False)

    def beep(self, n):
        if self.buzzer:
            msg = BuzzerState()
            msg.freq, msg.on_time, msg.off_time, msg.repeat = 2500, 0.08, 0.08, n
            self.buzzer.publish(msg)

    def pressed(self, msg, idx):
        now = idx < len(msg.buttons) and msg.buttons[idx] == 1
        was = self.prev.get(idx, False)
        self.prev[idx] = now
        return now and not was

    def on_joy(self, msg):
        if self.pressed(msg, self.btn_save):
            self.record()
        if self.pressed(msg, self.btn_undo) and self.checkpoints:
            removed = self.checkpoints.pop()
            self.save()
            self.beep(2)
            self.get_logger().info(f'Removed {removed["id"]}')

    def record(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.5))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'No {self.map_frame}->{self.base_frame} TF (localized?): {e}')
            return
        t = tf.transform.translation
        cp = {
            'id': f'cp_{len(self.checkpoints) + 1:02d}',
            'x': round(t.x, 3),
            'y': round(t.y, 3),
            'yaw': round(yaw_from_quat(tf.transform.rotation), 3),
        }
        self.checkpoints.append(cp)
        self.save()
        self.beep(1)
        self.get_logger().info(f'Saved {cp}')


def main():
    rclpy.init()
    node = CheckpointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
