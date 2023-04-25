import sys
import copy
import pygame
import os
import rclpy
import json
import requests

from georgebot_msgs.msg import ControllerFrame
from georgebot_msgs.srv import GenerateMap

from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String
from rclpy.node import Node

# Timer to run requesting controller and map info every 5 or whatever seconds
	# https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-publisher-node

# That timer will call both services to request a map image and the controller data
# The image needs to be saved to a certain path so it can be sent to the server
# The controller data needs to be turned into json data (see lidar_json.py) after the server has sent back the request to the client


class AWS_Client(Node):
	def __init__(self):
		super().__init__('aws_client')
		
		self.subscriber = self.create_subscription(ControllerFrame, 'controller', self.controller_callback, 5)
		self.map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
		
		#initialize the clients for each service
		#self.controller_client = self.create_client(ControllerQuery, 'controller') 
		#self.map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')        
		#put the client information into a JSON
		#if the service is not available we send an error message
		#while not self.controller_client.wait_for_service(timeout_sec=1.0):
		#	self.get_logger().info('Controller service not available, waiting again...')

		#while not self.map_client.wait_for_service(timeout_sec = 1.0):
			#self.get_logger().info('2D map not available, waiting again...')
		#get the request for a service

		self.timer_period = 5.0
		self.timer = self.create_timer(self.timer_period, self.request_data)

		self.map_counter = 0
		self.controller_frame = None
		#Send the data using a post request
		#self.req = requests.post("http://george-env.eba-trrm37cn.us-east-2.elasticbeanstalk.com/api/data-feed", controller_JSON)
		#self.req = requests.POST("http://george-env.eba-trrm37cn.us-east-2.elasticbeanstalk.com/api/data-feed", + self.cli_2)
	
	def controller_callback(self, msg):
		self.controller_frame = msg
	
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
		#for debugging purposes
		print(json.dumps(controller_json))

		
		file_path_full = os.path.join('/home/path/maps', ('map-%i' % self.map_counter))
		self.map_counter += 1

		map_request = SaveMap.Request()
		map_request.name = String(data=file_path_full)

		#self.cli.call_async(outgoing_request)

		# TODO: Send image over post
		
		files = {'media': open((file_path_full + ".pgm"), 'rb')}
		self.req = requests.post("http://george-env.eba-trrm37cn.us-east-2.elasticbeanstalk.com/api/data-feed", headers = headers, files)
		
		

def main():
	rclpy.init()
	client = AWS_Client()
	client.request_data()
	rclpy.spin(client)

	client.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
