import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource

# This is the launch file for running Georgebot with the intent of recording data to work on mapping offline
# Since the TF transforms are recorded as is with the bag, the time stamp will not be the same as what the mapping programs expect
# Because of that, we will be recording everything as nomral but omitting the odometry (this will be run offline later)

def generate_launch_description():
    controller_input = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('controller_input'), 'launch/'),
            'controller_input.launch.py'])
    )

    teleop_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('teleop_movement_control'), 'launch/'),
            'teleop_movement.launch.py'])
    )

    arduino_serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('arduino_serial'), 'launch/'),
            'arduino_serial.launch.py']),
        launch_arguments={'serial_port': '/dev/ttyACM0', 'baud_rate': '9600'}.items()
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('cyglidar_d1'), 'launch/'),
            'cyglidar.launch.py']),
        launch_arguments={'version': '0'}.items()
    )

    return LaunchDescription([
        controller_input,
        teleop_control,
        arduino_serial,
        lidar
    ])
