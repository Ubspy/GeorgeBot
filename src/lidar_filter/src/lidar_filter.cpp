#include <chrono>
#include <memory>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class LidarFilter : public rclcpp::Node
{
    public:
        LidarFilter() : Node("lidar_filter")
        {
            this->_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    "scan_2D", 10, std::bind(&LidarFilter::filter_pointcloud, this, std::placeholders::_1));
            
            this->_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "scan_2D_filtered", 10);
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscription;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;

        void filter_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
        {
            sensor_msgs::msg::PointCloud2 msg_out;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
            pcl::fromROSMsg(*msg, *cloud);

            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
            sor.setInputCloud(cloud);
            sor.setMeanK(50);
            sor.setStddevMulThresh(0.8);
            sor.filter(*cloud_filtered);

            pcl::toROSMsg(*cloud_filtered, msg_out);

            msg_out.header.stamp = this->now();

            // TODO: I messed up recording this doesn't need to be here
            msg_out.header.frame_id = "laser_link";

            _publisher->publish(msg_out);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFilter>());
    rclcpp::shutdown();
}
