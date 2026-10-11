from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    share = FindPackageShare("indoor_slam_sim")
    world = PathJoinSubstitution([share, "worlds", "indoor_house.world"])
    xacro_file = PathJoinSubstitution([share, "urdf", "mobile_base.urdf.xacro"])
    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_file])
    gazebo_launch = PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
    plugin_path = PathJoinSubstitution([FindPackagePrefix("indoor_slam_sim"), "lib"])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        SetEnvironmentVariable(
            "GAZEBO_PLUGIN_PATH",
            [plugin_path, ":", EnvironmentVariable("GAZEBO_PLUGIN_PATH", default_value="")],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={"world": world, "gui": LaunchConfiguration("gui"), "pause": "false"}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", "indoor_mobile_base",
                       "-x", "5.0", "-y", "5.0", "-z", "0.17", "-Y", "0.0"],
            output="screen",
        ),
        Node(
            package="indoor_slam_sim",
            executable="cmd_vel_limiter.py",
            output="screen",
            parameters=[{"use_sim_time": True, "max_linear": 0.4,
                         "max_angular": 0.8, "timeout_sec": 0.5}],
        ),
    ])
