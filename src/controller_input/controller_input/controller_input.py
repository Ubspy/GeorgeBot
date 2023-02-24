import rclpy
from rclpy.node import Node
from pygame import event, joystick

from controller_interface.msg import ControllerFrame

class ControllerPublisher(Node):
    def __init__(self):
        super().__init__('controller_publisher')
        self.publisher_ = self.create_publisher(ControllerFrame, 'controller', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.current_frame = ControllerFrame()

        # Initialize controller input
        joystick.init()
        self.controller = joystick.Joystick(0) # Get the 0th controller, I don't expect more than one

    def timer_callback(self):
        # TODO: Read controller inputs into current frame

        # We only want to publish when the controller values change, otherwise we're flooding the topic with info no one cares about
        if self.read_controller():
            # Publish current frame to the topic
            self.publisher_.publish(self.current_frame)
            self.get_logger().info("Published controller frame")

    def read_controller(self):
        # Save current frame to compare to last
        tmp_frame = self.current_frame

        # TODO: Read controller inputs and move them into self.prev_frame
        for currentEvent in event.get():
            match currentEvent.type:
                case JOYAXISMOTION:
                    match currentEvent.axis:
                        # Left Joystick
                        case 0:
                            self.current_frame.left_stick_x.value = currentEvent.value
                        case 1:
                            self.current_frame.left_stick_y.value = currentEvent.value
                        # Right Joystick
                        case 3:
                            self.current_frame.right_stick_x.value = currentEvent.value
                        case 4:
                            self.current_frame.right_stick_y.value = currentEvent.value
                        # Triggers
                        case 2:
                            self.current_frame.left_trigger.value = currentEvent.value
                        case 5:
                            self.current_frame.right_trigger.value = currentEvent.value
                case JOYBUTTONUP | JOYBUTTONDOWN:
                    match currentEvent.button:
                        # Main 4
                        case 0:
                            self.current_frame.a.value = currentEvent.value
                        case 1:
                            self.current_frame.b.value = currentEvent.value
                        case 2:
                            self.current_frame.x.value = currentEvent.value
                        case 3:
                            self.current_frame.y.value = currentEvent.value
                        # Bumpers
                        case 4:
                            self.current_frame.left_bumper.value = currentEvent.value
                        case 5:
                            self.current_frame.right_bumper.value = currentEvent.value
                        # Silly Buttons
                        case 6:
                            self.current_frame.select.value = currentEvent.value
                        case 7:
                            self.current_frame.start.value = currentEvent.value
                        case 10:
                            self.current_frame.menu.value = currentEvent.value
                        # Sticks
                        case 8:
                            self.current_frame.left_stick_down.value = currentEvent.value
                        case 9:
                            self.current_frame.right_stick_down.value = currentEvent.value

        # Return if our current frame is different from the previous one
        return tmp_frame == self.current_frame

def main(args=None):
    rclpy.init(args=args)
    controller_publisher = ControllerPublisher()
    rclpy.spin(controller_publisher)

    controller_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
