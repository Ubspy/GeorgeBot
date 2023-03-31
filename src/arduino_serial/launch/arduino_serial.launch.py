from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    # Set up arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value=TextSubstitution(text='/dev/ttyACM0')
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value=TextSubstitution(text='9600')
    )

    # Return launch description with our parameter arguments and the node running
    # with those parameter arguments
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        Node(
            package='arduino_serial',
            executable='bidirectional',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate')
            }]
        ),
    ])
