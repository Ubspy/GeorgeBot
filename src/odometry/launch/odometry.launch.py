from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    # Set up arguments
    wheel_diameter_arg = DeclareLaunchArgument(
        'wheel_diameter', default_value=TextSubstitution(text='100')
    )

    # Return launch description with our parameter arguments and the node running
    # with those parameter arguments
    return LaunchDescription([
        wheel_diameter_arg,
        Node(
            package='odometry',
            executable='encoders',
            parameters=[{
                'wheel_diamater' : LaunchConfiguration('wheel_diameter')
            }]
        ),
    ])
