#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot/visualization.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualization>());
    return 0;
}