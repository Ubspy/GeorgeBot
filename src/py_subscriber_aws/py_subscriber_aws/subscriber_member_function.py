# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import requests

from std_msgs.msg import bool


class SwitchSubscriber(Node):

    def __init__(self):
        super().__init__('switch_subscriber')
        
        #changed topic to 'test_switch and callback value to 5'
        self.subscription = self.create_subscription(
            bool,
            'test_switch',
            self.listener_callback,
            5)
        self.subscription  # prevent unused variable warning

	
    def listener_callback(self, msg):
        #get the data from the publisher and then send to the ec2 instance
        self.get_logger().info('I heard: "%s"' % msg.data)
        r = requests.get('http://george-env.eba-trrm37cn.us-east-2.elasticbeanstalk.com/api/test/' + str(msg.data))
        #get the website url and add the message contents to the url so it can be displayed on the website
    #https://george-env.eba-trrm37cn.us-east-2.elasticbeanstalk.com/api/test/*INSERT_ANYTHING_HERE*
  
    #    x = requests.get('http://ec2-18-218-120-112.us-east-2.compute.amazonaws.com/')
    #    r = requests.post( 'http://ec2-18-218-120-112.us-east-2.compute.amazonaws.com/',self.get_logger().info('EC2 heard: "%s"' % msg.data))
        
	
def main(args=None):
    rclpy.init(args=args)
    #changed name from minimal_subscriber to switchSubscriber
    switch_subscriber = SwitchSubscriber()

    rclpy.spin(switch_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    switch_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
