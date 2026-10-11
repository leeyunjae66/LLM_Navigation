#!/usr/bin/env python3
"""Bound /cmd_vel and stop the simulated robot when commands go stale."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelLimiter(Node):
    def __init__(self):
        super().__init__("cmd_vel_limiter")
        self.declare_parameter("max_linear", 0.4)
        self.declare_parameter("max_angular", 0.8)
        self.declare_parameter("timeout_sec", 0.5)
        self._max_linear = float(self.get_parameter("max_linear").value)
        self._max_angular = float(self.get_parameter("max_angular").value)
        self._timeout = float(self.get_parameter("timeout_sec").value)
        self._latest = Twist()
        self._last_command = 0.0
        self._publisher = self.create_publisher(Twist, "/cmd_vel_limited", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_command, 10)
        self.create_timer(0.02, self._publish)
        self.get_logger().info(
            f"limits: |linear.x|<={self._max_linear:.2f} m/s, "
            f"|angular.z|<={self._max_angular:.2f} rad/s, "
            f"watchdog={self._timeout:.1f}s"
        )

    def _on_command(self, msg: Twist):
        self._latest = Twist()
        self._latest.linear.x = max(-self._max_linear, min(self._max_linear, msg.linear.x))
        self._latest.angular.z = max(-self._max_angular, min(self._max_angular, msg.angular.z))
        self._last_command = time.monotonic()

    def _publish(self):
        command = Twist()
        if time.monotonic() - self._last_command <= self._timeout:
            command = self._latest
        self._publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
