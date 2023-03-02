from launch import LaunchDescription
from launch_ros.actions import Node

# Name of method ROS looks for for launching
def generate_launch_description():
    # Return a launch description with all the nodes we wish to run
    return LaunchDescription([
            # Chassis frame broadcaster
            Node(
                package='chassis_frame',
                executable='chassis_broadcast'
            ),
            # TODO: Idk how to actually set this transform up, maybe through slam_toolbox
            # Static broadcaster for map -> odom
            Node(
                package='tf2_ros',
                name='map_to_odom',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
            ),
            Node(
                package='tf2_ros',
                name='odom_to_base',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
            ),
            Node(
                package='tf2_ros',
                name='base_to_laser',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link']
            ),
        ])
