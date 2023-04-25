
import copy
import rclpy
import os
import rclpy


import georgebot_msgs.msg import ControllerFrame
from pygame import event, joystick
from rclpy.node import Node


class ControllerService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(ControllerFrame, 'controller', 10)
	
	timer_period = 0.05
	self.timer = self.create_timer(timer_period, self.timer_callback)
	
	
	os.environ["SDL_VIDEODRIVER"] = "dummy"
	pygame.init()
	pygame.display.init()
	joystick.init()
	
		
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
        # Full disclosure, I have no idea if we need to compare each individual element of the controller object
        # I did this because before it wasn't correctly showing that the frames were different
        # The issue ended up being that we needed a deep copy because it ended up using a reference to the previous controller frame
        # I'm kind of too scared to change this and it's gross but oh well
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
                # Main 4 buttons
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
        
def main():
    rclpy.init()
    controller_service = ControllerService()
    rclpy.spin(controller_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
