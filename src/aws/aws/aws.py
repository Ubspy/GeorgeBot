import os
import json
import requests
import rclpy
import time

from georgebot_msgs.msg import ControllerFrame
from PIL import Image
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String
from rcl_interfaces.msg import Log
from rclpy.node import Node

# Timer to run requesting controller and map info every 5 or whatever seconds
	# https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-publisher-node

# That timer will call both services to request a map image and the controller data
# The image needs to be saved to a certain path so it can be sent to the server

class AWS(Node):
    def __init__(self):
        super().__init__('aws_client')
		
        self.controller_subscriber = self.create_subscription(ControllerFrame, 'controller', self.controller_callback, 5)
        self.log_subscriber = self.create_subscription(Log, 'rosout', self.log_callback, 5)
        self.map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
		
		#initialize the clients for each service
		#self.controller_client = self.create_client(ControllerQuery, 'controller') 
		#put the client information into a JSON
		#if the service is not available we send an error message
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map generation service not available, waiting again...')

        self.timer_period = 5.0
        self.timer = self.create_timer(self.timer_period, self.request_data)

        self.map_counter = 0
        self.controller_frame = None
        self.next_log = ""
	
    def controller_callback(self, msg):
        self.controller_frame = msg

    def log_callback(self, msg):
        self.next_log += (msg.msg + "\n")
	
    def request_data(self):
        if self.controller_frame == None:
            return
		
        controller_frame = self.controller_frame

		# TODO: JSON this data (lidar_json.py)
        controller_json = {"left_stick_coords" : (controller_frame.left_stick_x.value, controller_frame.left_stick_y.value), \
							"left_stick_down" : str(controller_frame.left_stick_down.value), \
							"right_stick_coords": (controller_frame.right_stick_x.value, controller_frame.right_stick_y.value), \
							"right_stick_down" : str(controller_frame.right_stick_down.value), \
							"axis_left_trigger" : controller_frame.left_trigger.value, \
							"axis_right_trigger": controller_frame.right_trigger.value, \
							"left_bumper" : str(controller_frame.left_bumper.value), \
							"right_bumper": str(controller_frame.right_bumper.value), \
							"button_a": str(controller_frame.a.value), \
							"button_b": str(controller_frame.b.value), \
							"button_x": str(controller_frame.x.value), \
							"button_y": str(controller_frame.a.value), \
							"start": str(controller_frame.start.value), \
							"select": str(controller_frame.select.value), \
							"menu": str(controller_frame.menu.value) }
		
        controller_json = { "controller-log": (controller_json)}
        headers = { "Content-Type": 'application/json' }

        # Once it's fully JSON'd
        requests.post("http://george-env.eba-trrm37cn.us-east-2.elasticbeanstalk.com/api/data-feed", headers=headers, data=json.dumps(controller_json))
        self.get_logger().info("Controller data sent to AWS data-feed")

        self.next_log = self.next_log.replace('\'', '')[:700]
        log_json = { "ros-log" : self.next_log, }
        requests.post("http://george-env.eba-trrm37cn.us-east-2.elasticbeanstalk.com/api/data-feed", headers=headers, data=json.dumps(log_json))
        self.get_logger().info("ROS log data sent to AWS data-feed")
        self.next_log = ""

        file_path_full = os.path.join('/mnt/git/maps', ('map-%i' % self.map_counter))
        self.map_counter += 1

        map_request = SaveMap.Request()
        map_request.name = String(data=file_path_full)

        start_time = time.time()
        self.map_client.call_async(map_request)
        
        while not os.path.isfile((file_path_full + ".pgm")):
            if (time.time() - start_time) >= 4:
                return

        to_upload = file_path_full + ".png"
        os.system("convert {}.pgm {}.png".format(file_path_full, file_path_full))

        # Below is what's idea, a python portable solution but there's some gay ass error about
        # A buffer not being large enough, idk how to fix it so I replaced it with the above cringe
        # solution that works rn
        """
        image = None

        Image.MAX_IMAGE_PIXELS = None

        try:
            image = Image.open(file_path_full + ".pgm")
        except:
            return

        image.save(to_upload) 
        image.close()
        """

        files = [
            ('', (('map-%i' % self.map_counter), open(to_upload, 'rb'), 'image/png'))
        ]
        requests.request("POST", "http://george-env.eba-trrm37cn.us-east-2.elasticbeanstalk.com/api/2d-image", headers={}, data={}, files=files)

        self.get_logger().info("Map was sent to AWS data-feed")
        return
		
def main():
    rclpy.init()
    aws = AWS()
    rclpy.spin(aws)

    aws.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
