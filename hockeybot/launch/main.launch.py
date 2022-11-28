
"""Launch file for Franka robot."""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch node."""
    main = Node(
        package='hockeybot',
        executable='main',
        name='main_node',
    )
    Camera = Node(
        package='hockeybot',
        executable='cam_node',
        name='cam_node',
    )
    Trajectory = Node(
        package='hockeybot',
        executable='traj_calc',
        name='traj_node',
    )
    return LaunchDescription([
        main,
        Camera,
        Trajectory
    ])