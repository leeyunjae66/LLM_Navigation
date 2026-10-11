from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("indoor_slam_sim"), "config", "glim"])
    use_sim_time = {"use_sim_time": True}
    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=config),
        # GLIM's local map starts at the robot origin. The robot is spawned at world (5,5,0), yaw=0.
        # These fixed transforms register that SLAM-local origin into the requested world-aligned map frame.
        Node(package="tf2_ros", executable="static_transform_publisher", name="world_to_map",
             arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0",
                        "--frame-id", "world", "--child-frame-id", "map"],
             parameters=[use_sim_time], output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_slam_map",
             arguments=["--x", "5", "--y", "5", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0",
                        "--frame-id", "map", "--child-frame-id", "slam_map"],
             parameters=[use_sim_time], output="screen"),
        Node(package="glim_ros", executable="glim_rosnode", name="glim_ros",
             parameters=[{"use_sim_time": True, "config_path": LaunchConfiguration("config_path")}],
             output="screen"),
    ])
