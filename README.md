# LLM Task Planner (ROS 2 Humble)

ROS 2 workspace for translating natural-language commands into validated symbolic plans and dispatching navigation goals through Nav2.

## Workspace layout

- `src/llm_task_planner/llm_task_planner/`: planner, schema, validation, map, and Nav2 modules
- `src/llm_task_planner/config/semantic_map.yaml`: symbolic locations and example poses
- `src/llm_task_planner/launch/`: launch descriptions

The example coordinates are placeholders. Replace them with poses in your Nav2 map before robot use. The LLM should only return symbolic names; pose lookup and navigation remain deterministic Python responsibilities.

## Run the initial NAVIGATE flow

Start Nav2 and Ollama first. The planner defaults to Ollama at `http://localhost:11434` with model `qwen3:14b`; override `ollama_url`, `ollama_model`, or `semantic_map_path` with ROS parameters when needed. The model was selected after a small Korean command check that included an unmapped location.

```bash
ros2 launch llm_task_planner llm_task_planner.launch.py
```

Send a command from another sourced ROS 2 terminal:

```bash
ros2 topic pub --once /natural_language_command std_msgs/msg/String "{data: '책상으로 가줘'}"
```

The planner publishes a validated JSON object on `/task_plan`. The executor accepts only `NAVIGATE` targets present in the semantic map; `UNKNOWN` plans and invalid plans do not dispatch a Nav2 goal.

## Build

From this workspace root in a sourced ROS 2 Humble environment:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
