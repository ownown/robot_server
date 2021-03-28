#include "robot/sensors.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h> // usleep

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/sensors.hpp"

#include "BrickPi3/BrickPi3.h"

using namespace std::chrono_literals;

void SensorPublisher::timer_callback()
{
    this->bp->get_sensor(PORT_1, &(this->sensor_1));
    this->bp->get_sensor(PORT_2, &(this->sensor_2));
    this->bp->get_sensor(PORT_3, &(this->sensor_3));
    this->bp->get_sensor(PORT_4, &(this->sensor_4));
    auto message = robot_interfaces::msg::Sensors();
    message.sensor1 = this->sensor_1.cm;
    message.sensor2 = this->sensor_2.cm;
    message.sensor3 = this->sensor_3.cm;
    message.sensor4 = this->sensor_4.cm;

    RCLCPP_INFO(this->get_logger(), "Sensor 1: %.2fcm\tSensor 2: %.2fcm\tSensor 3: %.2fcm\tSensor 4: %.2fcm",
        message.sensor1, message.sensor2, message.sensor3, message.sensor4);
    this->publisher->publish(message);
}

SensorPublisher::SensorPublisher(std::shared_ptr<BrickPi3> bp) 
    : Node("Sensors"), bp(bp)
{
    RCLCPP_INFO(this->get_logger(), "SensorPublisher constructor");
    this->publisher = this->create_publisher<robot_interfaces::msg::Sensors>("sensors", 10);
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