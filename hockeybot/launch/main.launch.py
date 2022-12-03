
"""Launch file for Franka robot."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,Shutdown,ExecuteProcess,IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals,IfCondition,UnlessCondition
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution,Command,FindExecutable
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import yaml
import os

def generate_launch_description():
    """Launch node."""
    # Franka = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #     [os.path.join(get_package_share_directory('franka_moveit_config')),
    #     '/launch/moveit.launch.py']),
    # launch_arguments={
    #             'robot_ip': 'dont care',

    #         }.items())
    # use_fake_hardware = DeclareLaunchArgument(
    #     name = 'use_fake_hardware',
    #     default_value = 'false',
    #     choices= [
    #         'false',
    #         'true',
    #         'none'],
    #     description = 'flag to choose use Franka or Rviz'
    # )
    
    # ros2_control_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #      parameters=[robot_description, ros2_controllers_path],
    #     remappings=[('joint_states', 'franka/joint_states')],
    #     condition=LaunchConfigurationEquals('use_fake_hardware','false'),
    #             output={
    #         'stdout': 'screen',
    #         'stderr': 'screen',
    #     },
    # )
    # ros2_control_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[robot_description, ros2_controllers_path],
    #     remappings=[('joint_states', 'franka/joint_states')],
    #     condition=LaunchConfigurationEquals('use_fake_hardware','true'),
    #             output={
    #         'stdout': 'screen',
    #         'stderr': 'screen',
    #     },
    #     on_exit=Shutdown(),
    # )
    # ros2_control_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #      parameters=[robot_description, ros2_controllers_path],
    #     remappings=[('joint_states', 'franka/joint_states')],
    #     condition=LaunchConfigurationEquals('use_fake_hardware','none'),
    #             output={
    #         'stdout': 'screen',
    #         'stderr': 'screen',
    #     },
    # )
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
    simple_move_node = Node(
        package='moveit_helper',
        executable='simple_move',
        name='simple_mode',
    )
    return LaunchDescription([
        main,
        Camera,
        Trajectory,
        simple_move_node,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('franka_moveit_config')),
        '/launch/moveit.launch.py']),
        launch_arguments={'robot_ip': []}.items(),
        )
    ])