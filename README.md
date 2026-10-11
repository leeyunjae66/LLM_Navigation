# LLM Task Planner (ROS 2 Humble)

ROS 2 workspace for translating natural-language commands into validated symbolic plans and dispatching navigation goals through Nav2.

## Workspace layout

- `src/llm_task_planner/llm_task_planner/`: planner, schema, validation, map, and Nav2 modules
- `src/llm_task_planner/config/semantic_map.yaml`: symbolic locations and example poses
- `src/llm_task_planner/launch/`: launch descriptions

The example coordinates are placeholders. Replace them with poses in your Nav2 map before robot use. The LLM should only return symbolic names; pose lookup and navigation remain deterministic Python responsibilities.

## Build

From this workspace root in a sourced ROS 2 Humble environment:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
