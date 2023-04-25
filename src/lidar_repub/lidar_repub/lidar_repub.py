import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, LaserScan

class LidarRepublisher(Node):
    def __init__(self):
        super().__init__('lidar_republisher')

        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            'scan_2D',
            self.cloud_recv,
            100)

        self.cloud_publisher = self.create_publisher(PointCloud2, 'lidar_out', 100)

        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_recv,
            100)

        self.laser_publisher = self.create_publisher(LaserScan, 'scan_out', 100)

    def cloud_recv(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cloud_publisher.publish(msg)

    def laser_recv(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.laser_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_republisher = LidarRepublisher()
    rclpy.spin(lidar_republisher)

    lidar_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
