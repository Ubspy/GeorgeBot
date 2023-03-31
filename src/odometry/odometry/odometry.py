import math
import rclpy

from geometry_msgs.msg import TransformStamped
from georgebot_msgs.msg import IMUData
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')

        pc_diameter = ParameterDescriptor(description="Whe diameter of the robots wheels in cemtimeters")
        self.declare_parameter('wheel_diameter', 100, pc_diameter)

        self.subscription = self.create_subscription(
            IMUData,
            'imu_data',
            self.transform_frame,
            10
        )

        self.subscription

        self.tf_broadcaster = TransformBroadcaster(self)

        # TODO: This shouldn't be int
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().integer_value

    def transform_frame(self, msg):
        left_encoder_val = msg.x_encoder_left
        right_encoder_val = msg.x_encoder_right

        avg_encoder_val = (left_encoder_val + right_encoder_val) / 2
        x_displacement = (self.wheel_diameter * math.pi) * avg_encoder_val

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x_displacement
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    odometry = Odometry()

    rclpy.spin(odometry)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
