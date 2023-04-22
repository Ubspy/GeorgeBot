import copy
import pygame
import os
import rclpy

from georgebot_msgs.msg import ControllerFrame
from georgebot_msgs.srv import ControllerQuery
from pygame import event, joystick
from rclpy.node import Node


class ControllerPublisher(Node):
    def __init__(self):
        super().__init__('controller_publisher')
        self.publisher_ = self.create_publisher(ControllerFrame, 'controller', 10)
<<<<<<< Updated upstream

=======
        self.srv = self.create_service(ControllerQuery, 'controller', 10)
        # Create timer with period of 0.05 seconds, every period we call the callback function
>>>>>>> Stashed changes
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.current_frame = ControllerFrame()

        # Initialize controller input
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        pygame.display.init()
        joystick.init()
<<<<<<< Updated upstream
        self.controller = joystick.Joystick(0) # Get the 0th controller, I don't expect more than one

=======

        # Get the 0th controller, I don't expect more than one
        self.controller = joystick.Joystick(0) 
    def service_request(self):
        return(self.get_logger().Info('Incoming request\na:', self.current_frame))
        
        #self.get_logger().Info('Incoming request\na:', self.controller)
        
    
>>>>>>> Stashed changes
    def timer_callback(self):
        # We only want to publish when the controller values change, otherwise we're flooding the topic with info no one cares about
        if not self.read_controller():
            # Publish current frame to the topic
            self.publisher_.publish(self.current_frame)
            self.get_logger().info("Published controller frame")

    def read_controller(self):
        # Save current frame to compare to last
        # We need to use the copy library because if we use a direct assign then
        # both objects will be edited
        tmp_frame = copy.deepcopy(self.current_frame)

        # Read controller inputs and move them into self.prev_frame
        for currentEvent in event.get():
            if currentEvent.type == pygame.JOYAXISMOTION:
                # Left Joystick
                if currentEvent.axis == 0:
                    self.current_frame.left_stick_x.value = currentEvent.value
                elif currentEvent.axis == 1:
                    self.current_frame.left_stick_y.value = currentEvent.value
                # Right Joystick
                elif currentEvent.axis == 3:
                    self.current_frame.right_stick_x.value = currentEvent.value
                elif currentEvent.axis == 4:
                    self.current_frame.right_stick_y.value = currentEvent.value
                # Triggers
                elif currentEvent.axis == 2:
                    self.current_frame.left_trigger.value = currentEvent.value
                elif currentEvent.axis == 5:
                    self.current_frame.right_trigger.value = currentEvent.value
            elif currentEvent.type == pygame.JOYBUTTONUP or currentEvent.type == pygame.JOYBUTTONDOWN:
                # Main 4
                if currentEvent.button == 0:
                    self.current_frame.a.value = self.controller.get_button(0) == 1
                elif currentEvent.button == 1:
                    self.current_frame.b.value = self.controller.get_button(1) == 1
                elif currentEvent.button == 2:
                    self.current_frame.x.value = self.controller.get_button(2) == 1
                elif currentEvent.button == 3:
                    self.current_frame.y.value = self.controller.get_button(3) == 1
                # Bumpers
                elif currentEvent.button == 4:
                    self.current_frame.left_bumper.value = self.controller.get_button(4) == 1
                elif currentEvent.button == 5:
                    self.current_frame.right_bumper.value = self.controller.get_button(5) == 1
                # Silly Buttons
                elif currentEvent.button == 6:
                    self.current_frame.select.value = self.controller.get_button(6) == 1
                elif currentEvent.button == 7:
                    self.current_frame.start.value = self.controller.get_button(7) == 1
                elif currentEvent.button == 10:
                    self.current_frame.menu.value = self.controller.get_button(0) == 1
                # Sticks
                elif currentEvent.button == 8:
                    self.current_frame.left_stick_down.value = self.controller.get_button(8) == 1
                elif currentEvent.button == 9:
                    self.current_frame.right_stick_down.value = self.controller.get_button(9) == 1

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
