import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import lgpio

switchPin = 26

class SwitchPublisher(Node):
    # Constructor for publisher node
    def __init__(self):
        # Call parent constructor (why doesn't it do this be default?
        super().__init__('switch_publisher')

        # Set the publisher of this class to a new publisher that publishes a
        # message of type Bool, called "test_switch" and has a queue of 5
        self.publisher_ = self.create_publisher(Bool, 'test_switch', 5)

        # TODO: Make this node work with interrupts
        period = 0.5

        # Create a timer that calls the member function timer_callback once every period
        self.timer = self.create_timer(period, self.timer_callback)

        # Setup GPIO handler
        self.handler = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(self.handler, switchPin)

    def timer_callback(self):
        # Create a bool message and set it to if the switch is on or not
        msg = Bool()
        msg.data = bool(lgpio.gpio_read(self.handler, switchPin))

        # Publish the message
        self.publisher_.publish(msg)

        # Log the publishing of the message
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def destroy_node(self):
        super().destroy_node()
        lgpio.gpiochip_close(self.handler)

def main(args=None):
    rclpy.init(args=args)
    switch_publisher = SwitchPublisher()

    rclpy.spin(switch_publisher)

    switch_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
