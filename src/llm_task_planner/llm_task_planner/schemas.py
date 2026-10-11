"""Data contracts shared by the planner and skill executor."""

from dataclasses import asdict, dataclass
import json
from typing import Any


@dataclass(frozen=True)
class RobotPlan:
    skill: str
    target: str

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False)

    @classmethod
    def from_value(cls, value: Any) -> "RobotPlan":
        """Parse a plan object or JSON string without accepting extra fields."""
        if isinstance(value, str):
            value = json.loads(value)
        if not isinstance(value, dict):
            raise ValueError("Plan must be a JSON object")
        if set(value) != {"skill", "target"}:
            raise ValueError("Plan must contain exactly 'skill' and 'target'")
        skill, target = value["skill"], value["target"]
        if not isinstance(skill, str) or not isinstance(target, str):
            raise ValueError("'skill' and 'target' must be strings")
        return cls(skill=skill, target=target)
