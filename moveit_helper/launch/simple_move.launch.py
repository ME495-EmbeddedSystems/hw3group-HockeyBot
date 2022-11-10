from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

    simple_move_node = Node(
        package='moveit_helper',
        executable='simple_move',
        name='simple_mode',
    )

    return LaunchDescription([
        simple_move_node
    ])