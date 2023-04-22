from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    # Return launch description for the controller input node
    return LaunchDescription([
        Node(
            package='AWS_client',
            executable='AWS_client'
        )
    ])
