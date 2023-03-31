from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    # Return launch description for the controller input node
    return LaunchDescription([
        Node(
            package='controller_input',
            executable='controller'
        )
    ])
