#include "robot_rpi/sensors.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h> // usleep

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

void SensorPublisher::timer_callback()
{
    this->bp->get_sensor(PORT_1, &(this->sensor_1));
    this->bp->get_sensor(PORT_2, &(this->sensor_2));
    this->bp->get_sensor(PORT_3, &(this->sensor_3));
    this->bp->get_sensor(PORT_4, &(this->sensor_4));
    auto message = std_msgs::msg::String();
    message.data = "Sensor data :: 1: " + std::to_string(this->sensor_1.cm) +
        "cm 2: " + std::to_string(this->sensor_2.cm) + "cm 3: " +
        std::to_string(this->sensor_3.cm) + "cm 3: " +
        std::to_string(this->sensor_3.cm) + "cm 4: " +
        std::to_string(this->sensor_4.cm);
    RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());
    this->publisher->publish(message);
}

SensorPublisher::SensorPublisher(std::shared_ptr<BrickPi3> bp) 
    : Node("Sensors"), bp(bp)
{
    RCLCPP_INFO(this->get_logger(), "SensorPublisher constructor");
    this->publisher = this->create_publisher<std_msgs::msg::String>("sensors", 10);
    this->timer = this->create_wall_timer(
        20ms, std::bind(&SensorPublisher::timer_callback, this));
    
    int error = this->bp->detect();
    if (error)
    {
        RCLCPP_INFO(this->get_logger(), "Unable to detect BrickPi");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "BrickPi detected");
    }

    this->bp->set_sensor_type(PORT_1, SENSOR_TYPE_EV3_ULTRASONIC_CM);
    this->bp->set_sensor_type(PORT_2, SENSOR_TYPE_EV3_ULTRASONIC_CM);
    this->bp->set_sensor_type(PORT_3, SENSOR_TYPE_EV3_ULTRASONIC_CM);
    this->bp->set_sensor_type(PORT_4, SENSOR_TYPE_EV3_ULTRASONIC_CM);

    RCLCPP_INFO(this->get_logger(), "Sensor check loop");
    error = 4;
    while (error == 4)
    {
        error = 0;
        if (this->bp->get_sensor(PORT_1, &(this->sensor_1)))
        {
            error++;
        } 
        if (this->bp->get_sensor(PORT_2, &(this->sensor_2)))
        {
            error++;
        }
        if (this->bp->get_sensor(PORT_3, &(this->sensor_3)))
        {
            error++;
        }
        if (this->bp->get_sensor(PORT_4, &(this->sensor_4)))
        {
            error++;
        }
        RCLCPP_INFO(this->get_logger(), "Error: %d", error);
        if (error == 4) 
        {
            usleep(20000);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Loop finished");
}