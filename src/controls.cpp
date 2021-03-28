#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "BrickPi3/BrickPi3.h"
#include "robot_rpi/sensors.h"

int main(int argc, char **argv)
{
    std::shared_ptr<BrickPi3> bp = std::make_shared<BrickPi3>();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorPublisher>(bp));
    rclcpp::shutdown();
    
    bp->reset_all();
    return 0;
}
