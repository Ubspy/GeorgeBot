#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/detail/point_field__struct.hpp>
#include "lidarpoint.hpp"
#include "lidar_drawer.hpp"

using std::placeholders::_1;

class LidarPackager : public rclcpp::Node
{
    public:
        LidarPackager() : Node("lidar_packager")
        {
            _subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    "scan_2D", 10, std::bind(&LidarPackager::subCallback, this, _1));

            this->runOnce = false;
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscription;
        bool runOnce;

        void subCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
        {
            LidarPoint* lidarPoints = new LidarPoint[msg->width];

            sensor_msgs::msg::PointCloud2::_fields_type fields = msg->fields;
            this->get_logger();

            sensor_msgs::PointCloud2Iterator<float> x_iter(*msg, "x");

            int i = 0;
            for(x_iter; x_iter != x_iter.end(); ++x_iter)
            {
                lidarPoints[i] = LidarPoint(x_iter[0], x_iter[1], x_iter[2], 0);
                i++;
            }

            drawPointCloud(lidarPoints, msg->width, this->get_logger());

            delete[] lidarPoints;
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPackager>());
    rclcpp::shutdown();

    return 0;
}
