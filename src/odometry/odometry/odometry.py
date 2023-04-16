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

        # Parameter for wheel diameter
        pc_diameter = ParameterDescriptor(description="The diameter of the robots wheels in millimeters")
        self.declare_parameter('wheel_diameter', 100, pc_diameter)

        pc_error = ParameterDescriptor(description="The average percent of how much wheel power actually equates to movement")
        self.declare_parameter('wheel_error', 0.9, pc_error)

        # Set up subscription to imu_data
        self.subscription = self.create_subscription(
            IMUData,
            'imu_data',
            self.transform_frame,
            10
        )

        # Make a transform broadcaster so we can tell ROS how far the robot has moved
        self.tf_broadcaster = TransformBroadcaster(self)

        # Maybe change wheel diameter to not be an int? idk
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().integer_value
        self.wheel_error = self.get_parameter('wheel_error').get_parameter_value().double_value

    def transform_frame(self, msg):
        # The left encoder is reversed, since they spin opposite directions relative to the motors
        # TODO: Is there a better way to do this like a parameter?
        left_encoder_val = msg.x_encoder_left * -1
        right_encoder_val = msg.x_encoder_right
        front_encoder_val = msg.y_encoder_front * -1
        back_encoder_val = msg.y_encoder_back

        # Get the current angle of the robot
        theta = msg.yaw

        # Take average x encoder value
        avg_x_encoder_val = (left_encoder_val + right_encoder_val) / 2.0
        avg_y_encoder_val = (front_encoder_val + back_encoder_val) / 2.0

        # Calculate x displacement, first use the circumference formula with the known wheel's diameter
        # This would mean that one turn of a our wheel is equal to one circumference of the wheel (in ideal circumstances)
        # Then we need to divide that by how many times our encoder increases per one revolution
        # This should ideally convert our encoder turns to meters and we can use it in our transform
        # However, no wheel is perfect so we want a coeffecient of how much of the wheel spin equates to actual movement
        # Now thi should set us up perfectly but for some reason stuff in RVIZ moved way more than it was supposed to
        # Our lidar point clouds would barely be moving while the TF was flying across, so I divided by an extra factor of 10
        # TODO: Make some of this stuff into parameters
        x_displacement = (self.wheel_diameter / 1000.0 * math.pi) * avg_x_encoder_val / (48.0 * 10.0) * 0.85
        y_displacement = (self.wheel_diameter / 1000.0 * math.pi) * avg_y_encoder_val / (48.0 * 10.0) * 0.85
         
        # Now that we have our displacement create a tranform between odom and base_link based off what we just calculated
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

        # Log that we published our transform
        self.get_logger().info("Publishing x transform: %f" % x_displacement)
        self.get_logger().info("Publishing y transform: %f" % y_displacement)
        self.get_logger().info("Publishing angle transform: %f" % theta)

        # Send out transform to the transform broadcaster
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    odometry = Odometry()

    rclpy.spin(odometry)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
