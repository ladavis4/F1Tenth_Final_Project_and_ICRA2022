#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class NullNode : public rclcpp::Node {


    public:
        NullNode() : Node("null_node")
        {
            /// Create ROS subscribers and publishers
            drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1);
            scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 1, std::bind(&NullNode::null_callback, this, _1));
        }

    private:
        std::string lidarscan_topic = "/opp_scan";
        std::string drive_topic = "/opp_drive";

        /// Create ROS subscribers and publishers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;

        void null_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
        {
            // Publish Drive message
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.steering_angle = 0.0;
            drive_msg.drive.speed = 0.0;

            drive_publisher->publish(drive_msg);
        }
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NullNode>());
    rclcpp::shutdown();
    return 0;
}
