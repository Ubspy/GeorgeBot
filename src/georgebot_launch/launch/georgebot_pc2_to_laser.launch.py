from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# TODO: Maybe some launch arguments?

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan', node_executable='pointcloud_to_laserscan_node',
            # This package needs the incoming pointcloud2 to be named 'cloud_in' so we remap our pc2 to that name
            # The output is named 'scan' and we do a similar thing
            remappings=[('cloud_in', 'scan_2D_filtered'), ('scan', 'scan_filtered')],
            parameters=[{
                'target_frame': 'laser_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            node_name='pointcloud_to_laserscan'
        )
    ])
