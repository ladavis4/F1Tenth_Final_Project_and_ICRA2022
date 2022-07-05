#include "obs_detect_node/obs_detect_node.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OBS_DETECT>());
    rclcpp::shutdown();
    return 0;
}
