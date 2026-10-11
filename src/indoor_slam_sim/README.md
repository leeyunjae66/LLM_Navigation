# Indoor mobile robot + Livox + GLIM

This package adds a self-contained indoor Gazebo Classic world and a circular differential-drive robot to the LLM_Navigation ROS 2 Humble workspace. It is separate from `drok_gazebo`; that existing tracked robot package is not modified.

## Environment and sensor choices

- Tested environment: ROS 2 Humble, Gazebo Classic 11.10.2, ARM64.
- SLAM: GLIM ROS 2 CPU backend. This workspace had no installed FAST-LIO2 or GLIM; Nav2 server packages were also absent. GLIM was selected because its ROS 2 node accepts `sensor_msgs/PointCloud2` and `sensor_msgs/Imu`. Only GLIM is started.
- LiDAR: GPU depth-ray sensor (`gpu_ray`) resampled into the non-repeating angular distribution in the locally available Livox Mid-360 `mid360.csv` pattern. The plugin publishes `sensor_msgs/PointCloud2` on `/livox/lidar`, with `x`, `y`, `z`, and `intensity` fields. It samples 2,000 directions from the 800,000-row pattern at 10 Hz (nominal 20 k points/s), over 360 degrees horizontal, about -7.21 to +52.16 degrees vertical, and 0.15 to 30 m range. Gazebo first measures a 2048 x 128 uniform GPU grid, then samples the nearest grid range for each selected Mid-360 direction; this is GPU-accelerated and Livox-pattern-shaped, but angular quantization means it does not reproduce exact Livox rays.
- The GPU cloud is an instantaneous snapshot. It contains no point-wise timestamps, rolling scan distortion, or packet timing; GLIM is configured for global-shutter LiDAR and does not deskew. No artificial range noise is added. The cloud uses simulation time and `frame_id=livox_frame`.
- IMU: the package's Gazebo model plugin publishes physics-derived orientation, angular velocity and gravity-inclusive specific force on `/imu/data` as `sensor_msgs/Imu`, frame `imu_link`, at 200 Hz. It adds deterministic-seed Gaussian noise (0.0002 rad/s gyro, 0.002 m/s² accelerometer) and publishes matching diagonal covariance. It timestamps from Gazebo simulation time. This avoids the installed native IMU plugin's non-firing update callback observed in this model.
- Glass panes have both a transparent visual and a collision surface. Gazebo ray casts can detect the panes through collision geometry; this does not model real glass's optical or LiDAR reflection behavior.

## Map and room geometry

Gazebo's `world` origin is the lower-left corner of the 10 m by 20 m floor; x points right, y points up in a plan view, and z points up. Walls are 2.7 m high and 0.15 m thick. The ceiling is omitted. The large room occupies x=0..4, y=14..20, with an actual 1.4 m full-height opening at x=4, y=14.8..16.2. The small room occupies x=7.5..10, y=0..3, with an actual opening at x=7.5, y=0.8..2.2. Neither room has a window.

The living-room windows are actual wall openings: 2.5 m wide, from 0.8 to 2.3 m high, at x=3.5..6.0 on the bottom wall and y=10.5..13.0 on the right wall. Each has a simple frame and collision-backed transparent pane.

| Fixed object | Center (m) | Footprint / size (m) | Details |
| --- | --- | --- | --- |
| Sofa | (0.8, 9.0) | about 1.0 x 3.0 | Seat faces +x |
| TV stand | (7.3, 19.35) | 2.0 x 0.7 | Screen faces -y |
| Table | (5.2, 9.5) | 1.4 x 1.0 x 0.65 high | Tabletop and four legs |
| Orange box | (0.8, 19.2, 0.4) | 0.8 x 0.8 x 0.8 | Large-room corner |

No matching local furniture models were present in the LLM_Navigation workspace, so the sofa, TV, table, walls, frame, and box use low-detail static boxes with visual and collision geometry. The small room is empty. The closest sofa/table edges are 3.22 m apart.

The existing planner's semantic map contains robot-center goals in the aligned `map` frame. Goal coordinates are not object centers:

| Goal | Robot center (x, y, yaw rad) | Object center (x, y) |
| --- | --- | --- |
| sofa | (2.3, 9.0, 3.14159) | (0.8, 9.0) |
| tv | (7.3, 17.8, 1.57080) | (7.3, 19.35) |
| table | (5.2, 7.8, 1.57080) | (5.2, 9.5) |
| box | (0.8, 17.6, 1.57080) | (0.8, 19.2) |
| large_room | (2.0, 17.0, 0.0) | — |
| small_room | (8.75, 1.5, 0.0) | — |
| window_1 | (4.75, 1.2, -1.57080) | (4.75, 0.0) |
| window_2 | (8.8, 11.75, 0.0) | (10.0, 11.75) |

## SLAM frame registration and TF

GLIM estimates its own local `slam_map` frame and does not subscribe to Gazebo ground-truth pose. This launch starts the robot at `(5, 5, 0.17)`, yaw 0. It publishes two fixed registration transforms: `world -> map` is identity, and `map -> slam_map` is translation `(5, 5, 0)`, yaw 0. This registers GLIM's initial local origin to the configured start pose and aligns map axes to the world axes, so the existing goal coordinates can be used as `map` goals. This registration is valid for the configured spawn pose; changing the initial pose or yaw requires updating the registration.

The intended tree is:

```text
world -> map -> slam_map -> odom -> base_link
                                     ├── left_wheel_link
                                     ├── right_wheel_link
                                     ├── imu_link
                                     ├── livox_frame
                                     ├── front_caster_link
                                     └── rear_caster_link
```

`robot_state_publisher` owns the robot joints and sensor extrinsics. GLIM owns `slam_map -> odom -> base_link`; the wheel plugin publishes encoder odometry only on `/wheel/odom` and has odometry-TF publication disabled. `/wheel/odom` is separate and is not an input to SLAM. The initial spawn pose is used only for the fixed coordinate registration, never as a SLAM observation.

The LiDAR-to-IMU translation in GLIM is `[0, 0, 0.12, 0, 0, 0, 1]`: the LiDAR is 0.12 m above the IMU in the coincident sensor axes. The same extrinsics are defined by URDF joints; GLIM's additional `imu -> lidar` broadcaster is disabled to avoid a duplicate TF edge.

## Install dependencies

The machine this workspace was inspected on has ROS 2 Humble, Gazebo Classic 11.10.2, ARM64, and an NVIDIA Jetson AGX Orin. A locally built Gazebo ROS overlay is at `/home/kudos/gazebo_ros_ws` and provides the differential-drive plugin. The indoor package builds its own GPU ray and IMU model plugins.

On a fresh Ubuntu 22.04/ARM64 system, install ROS 2 Humble, Gazebo Classic 11, `gazebo_ros_pkgs`, `robot_state_publisher`, `xacro`, `teleop_twist_keyboard`, `rviz2`, `tf2_ros`, and GLIM's CPU ROS 2 package. GPU ray rendering needs a working OpenGL-capable GPU and a Gazebo rendering context. GLIM's official Ubuntu 22.04 ARM64 PPA instructions and source-build alternatives are at [GLIM installation](https://koide3.github.io/glim/installation.html). The ROS 2 node expects the GLIM and `gtsam_points` libraries to be installed and on the ROS/CMake library paths.

For this workstation, source the existing overlays before building/running:

```bash
source /opt/ros/humble/setup.bash
source /home/kudos/gazebo_ros_ws/install/setup.bash
cd ~/Documents/ChatGPT/LLM_Navigation
colcon build --symlink-install
source install/setup.bash
```

If GLIM is source-built outside this project's install tree, also source its overlay before `install/setup.bash`, or install GLIM's CPU dependencies as described in its official instructions. The local GLIM build used GTSAM 4.2 from ROS Humble and CPU-only modules; it did not require CUDA.

## Run the world and robot

```bash
source /opt/ros/humble/setup.bash
source /home/kudos/gazebo_ros_ws/install/setup.bash
cd ~/Documents/ChatGPT/LLM_Navigation
source install/setup.bash
ros2 launch indoor_slam_sim sim.launch.py
```

Keep the GUI enabled for GPU ray rendering (`gui:=true`, the default). The robot starts at `(5.0, 5.0)`, yaw 0. Both custom sensor plugins use simulation timestamps. Headless GPU rendering was not confirmed in this environment.

## Teleoperate and stop

In a second sourced terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/kudos/gazebo_ros_ws/install/setup.bash
source ~/Documents/ChatGPT/LLM_Navigation/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Use the keyboard teleop's `i`, `,`, `j`, and `l` motion keys and `k` to stop. `Ctrl-C` also stops teleop; the command limiter sends zero after 0.5 s without a fresh command. `/cmd_vel` is clamped to ±0.4 m/s and ±0.8 rad/s before the Gazebo drive plugin receives `/cmd_vel_limited`.

To command a short straight movement without keyboard teleop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
```

## Run GLIM and view topics

Keep Gazebo running. In another sourced terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/kudos/gazebo_ros_ws/install/setup.bash
source ~/Documents/ChatGPT/LLM_Navigation/install/setup.bash
ros2 launch indoor_slam_sim slam.launch.py
```

Open RViz in a third terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/kudos/gazebo_ros_ws/install/setup.bash
source ~/Documents/ChatGPT/LLM_Navigation/install/setup.bash
rviz2 -d "$(ros2 pkg prefix indoor_slam_sim)/share/indoor_slam_sim/rviz/indoor_slam.rviz"
```

Check topic type/rate and wheel odometry:

```bash
ros2 topic info /livox/lidar
ros2 topic info /imu/data
ros2 topic hz /livox/lidar
ros2 topic hz /imu/data
ros2 topic echo --once /wheel/odom
ros2 run tf2_tools view_frames
```

GLIM's CPU odometry/mapping result is published by its ROS 2 viewer extension. On shutdown, GLIM is configured to dump its mapping session under `/tmp/dump`. For replayable input, record `/clock`, `/tf`, `/tf_static`, `/livox/lidar`, `/imu/data`, and `/wheel/odom` with rosbag2.

## Goal and path geometry check

The provided checker rasterizes the world collision boxes at 5 cm resolution, inflates obstacles by the 0.25 m circular robot radius, and checks eight-connected reachability from the initial pose:

```bash
python3 src/indoor_slam_sim/scripts/check_goal_geometry.py
```

This geometric check does not replace a Nav2 planner. Nav2 server packages were not installed in the inspected workspace, so the deliverable includes teleoperation, LiDAR-IMU SLAM, and collision/path-clearance checks, not a Nav2 route execution test.

## Limitations

- GPU LiDAR range data is sampled from a regular 2048 x 128 grid into 2,000 directions selected from the Mid-360 CSV. It is a repeatable Livox-pattern-shaped approximation, not a true non-repeating Livox scan. Exact beam directions, point times, motion distortion, packet timing, Mid-360 reflectivity, and real glass returns are not simulated.
- A headless run and GUI run both spawned the robot and produced IMU data at about 199-200 Hz; a short `/cmd_vel` test moved the wheel odometry by 0.16 m and the explicit zero command stopped it. In both runs, this environment did not produce `/livox/lidar` from the GPU sensor callback, so LiDAR topic, GLIM mapping, TF fusion and SLAM map output remain unverified. Check the rendering stack on the Jetson before relying on SLAM.
- CPU GLIM mapping is computationally heavier than GPU builds. The point preprocessor downsamples to at most 5,000 points per frame for this Jetson-class workspace.
- The fixed map registration assumes the configured start pose `(5, 5, 0)`, yaw 0. If the robot spawn pose changes, update `slam.launch.py`'s `map -> slam_map` transform.
- The doors are clear collision gaps, but path clearance is only checked against static geometry; dynamic collision response, sensor dropout, SLAM drift, and localization stability require a live run.
