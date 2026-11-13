#!/usr/bin/env python3
"""Make sure map_server + AMCL end up ACTIVE.

On the loaded Jetson Nano, Nav2's lifecycle manager sometimes aborts the localization bringup
("Failed to change state for node: map_server. Exception: transition invoked while in
transition") and leaves AMCL inactive: no map -> odom TF, the robot never appears on the map
and every goal fails. This node waits until the normal bringup is over, then checks every few
seconds and finishes the job (configure / activate) for any node stuck in a non-active state.
"""
import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class LifecycleGuard(Node):
    def __init__(self):
        super().__init__('lifecycle_guard')
        self.nodes = self.declare_parameter('nodes', ['map_server', 'amcl']).value
        self.start_delay = self.declare_parameter('start_delay', 45.0).value
        period = self.declare_parameter('period', 5.0).value
        self.get = {n: self.create_client(GetState, f'/{n}/get_state') for n in self.nodes}
        self.set = {n: self.create_client(ChangeState, f'/{n}/change_state') for n in self.nodes}
        self.strikes = {n: 0 for n in self.nodes}
        self.busy = set()
        self.t0 = self.get_clock().now()
        self.create_timer(period, self.check)

    def check(self):
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 < self.start_delay:
            return
        for n in self.nodes:
            if n in self.busy or not self.get[n].service_is_ready():
                continue
            self.busy.add(n)
            self.get[n].call_async(GetState.Request()).add_done_callback(
                lambda f, n=n: self.on_state(n, f))

    def on_state(self, n, fut):
        self.busy.discard(n)
        st = fut.result().current_state.id if fut.result() else None
        if st == State.PRIMARY_STATE_ACTIVE or st is None:
            self.strikes[n] = 0
            return
        self.strikes[n] += 1
        if self.strikes[n] < 2:          # seen twice in a row (not mid-transition) -> act
            return
        tr = {State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
              State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE}.get(st)
        if tr is None:
            return
        self.get_logger().warn(f'{n} stuck in state {st}: sending '
                               f'{"configure" if tr == Transition.TRANSITION_CONFIGURE else "activate"}')
        req = ChangeState.Request()
        req.transition.id = tr
        self.busy.add(n)
        self.set[n].call_async(req).add_done_callback(lambda f, n=n: self.busy.discard(n))
        self.strikes[n] = 0


def main():
    rclpy.init()
    node = LifecycleGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
