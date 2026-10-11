#!/usr/bin/env python3
"""Check goal footprint clearance and 8-connected reachability on a 5 cm grid."""

import math
import sys
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "worlds" / "indoor_house.world"
GOALS = ROOT.parent / "llm_task_planner" / "config" / "semantic_map.yaml"
RESOLUTION = 0.05
RADIUS = 0.25


def boxes_from_world():
    sdf = ET.parse(WORLD).getroot()
    model = sdf.find("world/model")
    boxes = []
    for link in model.findall("link"):
        if link.get("name", "").startswith("floor_"):
            continue
        pose = [float(v) for v in link.findtext("pose", "0 0 0 0 0 0").split()]
        for collision in link.findall("collision"):
            size = collision.find("geometry/box/size")
            if size is None:
                continue
            sx, sy, _ = (float(v) for v in size.text.split())
            boxes.append((pose[0], pose[1], sx, sy))
    return boxes


def clear(x, y, boxes):
    radius2 = RADIUS * RADIUS
    for cx, cy, sx, sy in boxes:
        dx = max(abs(x - cx) - sx / 2.0, 0.0)
        dy = max(abs(y - cy) - sy / 2.0, 0.0)
        if dx * dx + dy * dy < radius2 - 1e-12:
            return False
    return 0.25 <= x <= 9.75 and 0.25 <= y <= 19.75


def main():
    boxes = boxes_from_world()
    data = yaml.safe_load(GOALS.read_text())
    goals = data["locations"]
    xs = round(10 / RESOLUTION) + 1
    ys = round(20 / RESOLUTION) + 1
    free = [[False] * ys for _ in range(xs)]
    for ix in range(xs):
        for iy in range(ys):
            free[ix][iy] = clear(ix * RESOLUTION, iy * RESOLUTION, boxes)

    def cell(pose):
        return round(float(pose["x"]) / RESOLUTION), round(float(pose["y"]) / RESOLUTION)

    start = cell({"x": 5.0, "y": 5.0})
    queue = deque([start])
    reached = {start}
    while queue:
        x, y = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < xs and 0 <= ny < ys) or not free[nx][ny]:
                continue
            if dx and dy and (not free[x + dx][y] or not free[x][y + dy]):
                continue
            if (nx, ny) not in reached:
                reached.add((nx, ny))
                queue.append((nx, ny))

    failed = False
    print(f"grid={RESOLUTION:.2f}m robot_radius={RADIUS:.2f}m obstacles={len(boxes)}")
    for name, pose in goals.items():
        p = cell(pose)
        valid = free[p[0]][p[1]] and p in reached
        failed |= not valid
        # Minimum Euclidean distance between the robot center and any obstacle box.
        margin = min(math.hypot(max(abs(pose["x"] - cx) - sx / 2, 0),
                                max(abs(pose["y"] - cy) - sy / 2, 0)) - RADIUS
                     for cx, cy, sx, sy in boxes)
        print(f"{name:10s} {'PASS' if valid else 'FAIL'} center=({pose['x']:.2f},{pose['y']:.2f}) "
              f"nearest_clearance={margin:.2f}m path={'reachable' if p in reached else 'blocked'}")
    print(f"connected_free_cells={len(reached)}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
