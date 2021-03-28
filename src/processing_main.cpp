#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "robot/processing.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Processing>());
  rclcpp::shutdown();

  return 0;
}
