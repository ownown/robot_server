#include "robot/processing.h"

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/sensors.hpp"

using std::placeholders::_1;

void Processing::topic_callback(const robot_interfaces::msg::Sensors::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Sensor 1: %.2fcm\tSensor 2: %.2fcm\tSensor 3: %.2fcm\tSensor 4: %.2fcm",
        msg->sensor1, msg->sensor2, msg->sensor3, msg->sensor4);
}

Processing::Processing() : Node("processing")
{
    this->subscription = this->create_subscription<robot_interfaces::msg::Sensors>(
        "sensors", 10, std::bind(&Processing::topic_callback, this, _1));
}
