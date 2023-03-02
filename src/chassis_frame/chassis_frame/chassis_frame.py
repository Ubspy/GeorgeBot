import math
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ChassisFrameBroadcaster(Node):
    def __init__(self):
        # Name node
        super().__init__('chassis_frame_broadcaster')
        
        # Create the frame braodcaster object
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the position node that gets the current robot position from encoders and such
        # TODO: Create message obj
        #self.subscription = self.create_subscription(RobotPos, 'robot-pos', self.set_robot_pos, 5)

        self.timer = self.create_timer(0.1, self.set_robot_pos)

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

    def set_robot_pos(self):
        # Create a time stamped transform object
        t = TransformStamped()

        # Stamp the header with the current time
        t.header.stamp = self.get_clock().now().to_msg()
        
        # Set the parent as the static 'odom' frame 
        t.header.frame_id = 'odom'

        # Set the child frame to the 'odom' frame 
        t.child_frame_id = 'base_link'

        # Give it an initial position at the origin
        # TODO: Get variables from message type
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        q = self.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    chassis_frame_broadcaster = ChassisFrameBroadcaster()
    rclpy.spin(chassis_frame_broadcaster)

    chassis_frame_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
