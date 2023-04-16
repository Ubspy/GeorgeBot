import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource

# This is the launch file for when re've recorded data from the robot and wish to work on the mapping without the actual robot present
# Since we aren't actively controlling the robot, we don't need any of the nodes for movement control
# Odometry is needed because we need to have the timestamps on the TF be the same as the current time stamp
# TODO: Add lidar republishing

def generate_launch_description():
    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('odometry'), 'launch/'),
            'odometry.launch.py']),
        launch_arguments={'wheel_diameter': '10', 'wheel_error': '0.9', 'encoder_rev_count': '48'}.items()
    )

    return LaunchDescription([
        odometry,
        Node(
            package='tf2_ros',
            name='odom_to_map',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        Node(
            package='tf2_ros',
            name='base_to_odom',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
        Node(
            package='tf2_ros',
            name='laser_to_base',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.25', '0', '0', '0.707', '0.707', 'base_link', 'laser_link']
        ), 
    ])
