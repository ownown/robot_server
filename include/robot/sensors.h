#ifndef OWN_SENSORS_H
#define OWN_SENSORS_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/sensors.hpp"

#include "BrickPi3/BrickPi3.h"

class SensorPublisher : public rclcpp::Node
{
private:
    std::shared_ptr<BrickPi3> bp;
    sensor_ultrasonic_t sensor_1;
    sensor_ultrasonic_t sensor_2;
    sensor_ultrasonic_t sensor_3;
    sensor_ultrasonic_t sensor_4;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<robot_interfaces::msg::Sensors>::SharedPtr publisher;

    void timer_callback();
public:
    SensorPublisher(std::shared_ptr<BrickPi3> bp);
};

#endif // OWN_SENSORS_H
