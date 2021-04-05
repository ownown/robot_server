#include "rclcpp/rclcpp.hpp"

#include "robot/processing.h"

#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server ready");
    rclcpp::spin(std::make_shared<Processing>());
    rclcpp::shutdown();

    return 0;
}
