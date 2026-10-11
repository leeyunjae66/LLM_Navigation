"""ROS 2 node: natural-language command to validated symbolic task plan."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import rclpy
from std_msgs.msg import String

from .llm_client import OllamaClient
from .prompt_builder import build_prompt
from .schemas import RobotPlan
from .semantic_map import SemanticMap
from .validator import PlanValidationError, validate_plan


class PlannerNode(Node):
    def __init__(self):
        super().__init__("llm_planner")
        default_map = str(Path(get_package_share_directory("llm_task_planner"))
                          / "config" / "semantic_map.yaml")
        self.declare_parameter("semantic_map_path", default_map)
        self.declare_parameter("ollama_url", "http://localhost:11434")
        self.declare_parameter("ollama_model", "qwen2.5:7b")
        self.semantic_map = SemanticMap(
            self.get_parameter("semantic_map_path").get_parameter_value().string_value
        )
        self.llm = OllamaClient(
            self.get_parameter("ollama_url").value,
            self.get_parameter("ollama_model").value,
        )
        self.publisher = self.create_publisher(String, "/task_plan", 10)
        self.subscription = self.create_subscription(
            String, "/natural_language_command", self._on_command, 10
        )
        self.get_logger().info("Listening on /natural_language_command")

    def _on_command(self, message: String) -> None:
        command = message.data.strip()
        if not command:
            self.get_logger().warning("Ignoring empty natural-language command")
            return
        try:
            prompt = build_prompt(command, self.semantic_map.get_locations())
            raw_plan = self.llm.generate_plan(prompt)
            plan = validate_plan(
                RobotPlan.from_value(raw_plan),
                set(self.semantic_map.get_locations()),
            )
        except (RuntimeError, ValueError, PlanValidationError) as exc:
            self.get_logger().error(f"Plan rejected; no task plan published: {exc}")
            return
        output = String()
        output.data = plan.to_json()
        self.publisher.publish(output)
        self.get_logger().info(f"Published plan: {output.data}")


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
