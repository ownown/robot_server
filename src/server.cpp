#include "rclcpp/rclcpp.hpp"

#include "robot_server/processing.h"

#include <memory>
#include <string>

int main(int argc, char **argv)
{
    // If relative, from workspace root
    const std::string kPropertiesFileName = "./src/robot_server/robot_properties.yml";

    rclcpp::init(argc, argv);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server starting");
    rclcpp::spin(std::make_shared<Processing>(kPropertiesFileName));
    rclcpp::shutdown();

    return 0;
}
