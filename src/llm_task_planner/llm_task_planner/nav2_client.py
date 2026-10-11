"""Nav2 NavigateToPose action wrapper."""

import math
from typing import Callable

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class Nav2Client:
    def __init__(self, node: Node, action_name: str = "navigate_to_pose"):
        self.node = node
        self._client = ActionClient(node, NavigateToPose, action_name)

    def navigate(
        self,
        x: float,
        y: float,
        yaw: float,
        on_complete: Callable[[bool], None],
    ) -> None:
        """Send a goal without blocking the executor's ROS event loop."""
        if not self._client.server_is_ready():
            self.node.get_logger().error("Nav2 NavigateToPose action is unavailable")
            on_complete(False)
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future: self._on_goal_response(future, on_complete)
        )

    def _on_goal_response(self, future, on_complete: Callable[[bool], None]) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001 - report middleware failures to the node
            self.node.get_logger().error(f"Failed to send Nav2 goal: {exc}")
            on_complete(False)
            return
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("Nav2 rejected the navigation goal")
            on_complete(False)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_navigation_result(result, on_complete)
        )

    def _on_navigation_result(self, future, on_complete: Callable[[bool], None]) -> None:
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - report middleware failures to the node
            self.node.get_logger().error(f"Failed to receive Nav2 result: {exc}")
            on_complete(False)
            return
        succeeded = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        if not succeeded:
            status = "unknown" if result is None else str(result.status)
            self.node.get_logger().warning(f"Navigation did not succeed (status={status})")
        on_complete(succeeded)
