#!/usr/bin/env python3
"""Xbox controller teleop for the JetAuto (mecanum base).

Safety model
  * Robot only moves while LB (enable) or RB (turbo) is held  -> deadman switch.
  * Releasing the deadman sends a single zero Twist, then the node goes silent,
    so it never fights Nav2 for the motors during autonomous navigation.
  * B = emergency stop (latched). START clears the latch.
  * If /joy goes stale (controller disconnected) while moving -> zero Twist.

Default axis/button indices match the `game_controller_node` (SDL standard
layout). For `joy_node` use config/xbox_joy_node.yaml instead.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class XboxTeleop(Node):
    def __init__(self):
        super().__init__('xbox_teleop')
        p = self.declare_parameter
        # Axes
        self.ax_lin_x = p('axis_linear_x', 1).value     # left stick up/down
        self.ax_lin_y = p('axis_linear_y', 0).value     # left stick left/right (strafe)
        self.ax_ang_z = p('axis_angular_z', 2).value    # right stick left/right
        # Buttons
        self.btn_enable = p('button_enable', 9).value   # LB
        self.btn_turbo = p('button_turbo', 10).value    # RB
        self.btn_estop = p('button_estop', 1).value     # B
        self.btn_reset = p('button_reset', 6).value     # START
        # Speeds (m/s, rad/s)
        self.lin = p('scale_linear', 0.15).value
        self.lin_turbo = p('scale_linear_turbo', 0.30).value
        self.ang = p('scale_angular', 0.5).value
        self.ang_turbo = p('scale_angular_turbo', 1.0).value
        self.enable_strafe = p('enable_strafe', True).value
        self.deadzone = p('deadzone', 0.1).value
        self.joy_timeout = p('joy_timeout', 0.5).value
        topic = p('cmd_vel_topic', 'controller/cmd_vel').value

        self.pub = self.create_publisher(Twist, topic, 1)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_timer(0.05, self.tick)  # 20 Hz

        self.joy = None
        self.last_joy_time = None
        self.moving = False
        self.estop = False
        self.get_logger().info(
            f'Xbox teleop ready -> {topic}. Hold LB to drive, RB for turbo, '
            'B = E-STOP, START = clear E-STOP.')

    @staticmethod
    def _get(arr, i, default=0.0):
        return arr[i] if 0 <= i < len(arr) else default

    def _axis(self, i):
        v = float(self._get(self.joy.axes, i))
        return 0.0 if abs(v) < self.deadzone else v

    def _btn(self, i):
        return bool(self._get(self.joy.buttons, i, 0))

    def on_joy(self, msg):
        self.joy = msg
        self.last_joy_time = self.get_clock().now()
        if self._btn(self.btn_estop) and not self.estop:
            self.estop = True
            self.get_logger().warn('E-STOP latched (press START to clear)')
            self.stop()
        if self._btn(self.btn_reset) and self.estop:
            self.estop = False
            self.get_logger().info('E-STOP cleared')

    def stop(self):
        self.pub.publish(Twist())
        self.moving = False

    def tick(self):
        if self.joy is None:
            return
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        turbo = self._btn(self.btn_turbo)
        enabled = (self._btn(self.btn_enable) or turbo) and not self.estop
        if age > self.joy_timeout:
            if self.moving:
                self.get_logger().warn('Joystick timed out -> stopping')
                self.stop()
            return
        if not enabled:
            if self.moving:
                self.stop()
            return
        lin = self.lin_turbo if turbo else self.lin
        ang = self.ang_turbo if turbo else self.ang
        t = Twist()
        t.linear.x = lin * self._axis(self.ax_lin_x)
        if self.enable_strafe:
            t.linear.y = lin * self._axis(self.ax_lin_y)
        t.angular.z = ang * self._axis(self.ax_ang_z)
        self.pub.publish(t)
        self.moving = True


def main():
    rclpy.init()
    node = XboxTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
