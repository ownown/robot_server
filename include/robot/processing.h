#ifndef OWN_PROCESSING_H
#define OWN_PROCESSING_H

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/sensors.hpp"

class Processing : public rclcpp::Node
{
private:
    rclcpp::Subscription<robot_interfaces::msg::Sensors>::SharedPtr subscription;
    void topic_callback(const robot_interfaces::msg::Sensors::SharedPtr msg) const;
public:
    Processing();
};

#endif // OWN_PROCESSING_H
