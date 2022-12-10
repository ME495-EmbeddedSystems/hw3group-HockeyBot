"""Launch file for Franka robot."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    """Launch node and moveit/rviz."""
    DeclareLaunchArgument(
        name="robot",
        default_value="none",
        choices=["true", "false", "none"],
        description="Determines if moveit.launch.py is called.",
    )
    main = Node(
        package="hockeybot",
        executable="main",
        name="main_node",
    )
    Camera = Node(
        package="hockeybot",
        executable="cam_node",
        name="cam_node",
    )
    Trajectory = Node(
        package="hockeybot",
        executable="traj_calc",
        name="traj_node",
    )
    simple_move_node = Node(
        package="moveit_helper",
        executable="simple_move",
        name="simple_mode",
    )
    return LaunchDescription(
        [
            main,
            Camera,
            Trajectory,
            simple_move_node,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("franka_moveit_config")
                        ),
                        "/launch/moveit.launch.py",
                    ]
                ),
                launch_arguments={
                    "robot_ip": "dont-care",
                    "use_fake_hardware": "true",
                }.items(),
                condition=LaunchConfigurationEquals(
                    "robot", "false"
                ),  # launch moveit.launch.py
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("franka_moveit_config")
                        ),
                        "/launch/rviz.launch.py",
                    ]
                ),
                launch_arguments={"robot_ip": "panda0.robot"}.items(),
                condition=LaunchConfigurationEquals(
                    "robot", "true"
                ),  # launch rviz.launch.py
            ),
        ]
    )
