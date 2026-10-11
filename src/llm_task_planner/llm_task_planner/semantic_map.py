"""Load symbolic locations and poses from semantic_map.yaml."""

from pathlib import Path
from typing import Any

import yaml


class SemanticMap:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        with self.path.open(encoding="utf-8") as stream:
            data: Any = yaml.safe_load(stream)
        if not isinstance(data, dict) or not isinstance(data.get("locations"), dict):
            raise ValueError("semantic map must contain a 'locations' mapping")
        self._locations = data["locations"]
        for name, pose in self._locations.items():
            if not isinstance(name, str) or not isinstance(pose, dict):
                raise ValueError("each location must map a string name to a pose")
            for key in ("x", "y", "yaw"):
                if key not in pose or not isinstance(pose[key], (int, float)):
                    raise ValueError(f"location {name!r} requires numeric {key}")

    def get_locations(self) -> dict[str, str]:
        return {
            name: str(value.get("description", name))
            for name, value in self._locations.items()
        }

    def has_location(self, name: str) -> bool:
        return name in self._locations

    def get_pose(self, name: str) -> dict[str, float]:
        if not self.has_location(name):
            raise KeyError(f"Unknown semantic location: {name}")
        pose = self._locations[name]
        return {key: float(pose[key]) for key in ("x", "y", "yaw")}
