import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class LidarJson(Node):
    def __init__(self):
        super().__init__('lidar_json')
        self.subscription = self.create_subscription(
                PointCloud2,
                'scan_3D',
                self.lisener_callback,
                10)
        self.subscription

        self.count = 0

    def lisener_callback(self, msg):
        # Create a data dict for the lidar frame
        data = {}

        # Header
        header = {}
        time_num = msg.header.stamp.sec + (msg.header.stamp.nanosec / pow(10, 9))
        header["stamp"] = time_num
        header["frame_id"] = msg.header.frame_id

        data["header"] = header

        # PointCloud Fields 
        fields = []

        for field in msg.fields:
            fieldDict = {}
            fieldDict["name"] = field.name
            fieldDict["offset"] = field.offset
            fieldDict["datatype"] = field.datatype
            fieldDict["count"] = field.count
            fields.append(fieldDict)

        data["fields"] = list(fields)

        # Individual data fields
        data["width"] = msg.width
        data["height"] = msg.height
        data["is_bigendian"] = msg.is_bigendian
        data["point_step"] = msg.point_step
        data["row_step"] = msg.row_step
        data["data"] = msg.data.tolist()
        data["is_dense"] = msg.is_dense

        # TODO: This should be published to aws
        # TODO: Perhaps send multiple frames at a time?
        # Output json
        file_name = 'json/lidar_frame_%i.json' % self.count
        with open(file_name, 'w') as f:
            json.dump(data, f)

        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    lidar_json = LidarJson()
    rclpy.spin(lidar_json)

    lidar_json.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
