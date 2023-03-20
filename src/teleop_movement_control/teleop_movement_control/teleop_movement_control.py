import rclpy

from georgebot_msgs.msg import ControllerFrame 
from georgebot_msgs.msg import Direction
from rclpy.node import Node

class TeleopMovementController(Node):
    def __init__(self):
        # Setup node name
        super().__init__('teleop_movement_controller')

        # Create subscriber to controller frame
        self.subscription = self.create_subscription(
                ControllerFrame,
                'controller',
                self.process_controller_input,
                10)

        # Suppress unused warning
        self.subscription

        # Create publisher to publish movement data
        self.publisher = self.create_publisher(Direction, 'move_direction', 10)

    def process_controller_input(self, msg):
        # For the current implementation, the X and Y values for movement will come from
        # the X and Y axis from the left joystick
        local_x = msg.left_stick_x.value
        local_y = msg.left_stick_y.value

        # The turning will be from the right joystick X value
        local_theta = msg.right_stick_x.value

        # Create new movement message
        to_send = Direction()

        # Set values
        to_send.x = local_x
        to_send.y = local_y
        to_send.theta = local_theta

        self.get_logger().info('SENDING MOVEMENT')

        # Publish direction message
        self.publisher.publish(to_send)

def main(args=None):
    rclpy.init(args=args)
    teleop_movement_controller = TeleopMovementController()

    rclpy.spin(teleop_movement_controller)

    teleop_movement_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
