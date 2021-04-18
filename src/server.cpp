#include "rclcpp/rclcpp.hpp"

#include "robot/processing.h"

#include <memory>
#include <string>

int main(int argc, char **argv)
{
    const std::string kPropertiesFileName = "robot_properties.yml";

    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server ready");
    rclcpp::spin(std::make_shared<Processing>(kPropertiesFileName));
    rclcpp::shutdown();

    return 0;
}
