"""ROS 2 node: validated symbolic plan to deterministic Nav2 dispatch."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import rclpy
from std_msgs.msg import String

from .nav2_client import Nav2Client
from .schemas import RobotPlan
from .semantic_map import SemanticMap
from .validator import PlanValidationError, validate_plan


class ExecutorNode(Node):
    def __init__(self):
        super().__init__("skill_executor")
        default_map = str(Path(get_package_share_directory("llm_task_planner"))
                          / "config" / "semantic_map.yaml")
        self.declare_parameter("semantic_map_path", default_map)
        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.semantic_map = SemanticMap(
            self.get_parameter("semantic_map_path").value
        )
        self.nav2 = Nav2Client(self, self.get_parameter("navigate_action").value)
        self._navigation_active = False
        self.subscription = self.create_subscription(
            String, "/task_plan", self._on_plan, 10
        )
        self.get_logger().info("Listening on /task_plan")

    def _on_plan(self, message: String) -> None:
        try:
            plan = validate_plan(
                RobotPlan.from_value(message.data),
                set(self.semantic_map.get_locations()),
            )
        except (ValueError, PlanValidationError) as exc:
            self.get_logger().error(f"Ignoring invalid task plan: {exc}")
            return
        if plan.skill == "UNKNOWN":
            self.get_logger().warning("Planner returned UNKNOWN; no action dispatched")
            return
        if self._navigation_active:
            self.get_logger().warning("Ignoring plan while a navigation goal is active")
            return
        pose = self.semantic_map.get_pose(plan.target)
        self.get_logger().info(f"Navigating to semantic target: {plan.target}")
        self._navigation_active = True
        self.nav2.navigate(
            **pose,
            on_complete=lambda succeeded: self._on_navigation_complete(
                plan.target, succeeded
            ),
        )

    def _on_navigation_complete(self, target: str, succeeded: bool) -> None:
        self._navigation_active = False
        if succeeded:
            self.get_logger().info(f"Reached target: {target}")
        else:
            self.get_logger().warning(f"Navigation failed for target: {target}")


def main(args=None):
    rclpy.init(args=args)
    node = ExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
