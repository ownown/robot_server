/**
 * @file processing.h
 * @author Oliver Newman (oliver@olivernewman.co.uk)
 * @brief 
 * @version 0.1
 * @date 2021-04-05
 * 
 * (c) Copyright 2021 Oliver Newman
 * 
 */
#ifndef OWN_PROCESSING_H
#define OWN_PROCESSING_H

#include "rclcpp/rclcpp.hpp"

#include "robot/srv/robot_control.hpp"
#include "robot/msg/motors.hpp"
#include "robot/msg/sensors.hpp"

using ReqPtr = std::shared_ptr<robot::srv::RobotControl::Request>;
using ResPtr = std::shared_ptr<robot::srv::RobotControl::Response>;

class Processing : public rclcpp::Node
{
private:
    const int kNormalSpeed  = 330;
    const int kTurningSpeed = 110;
    const float kThreshold  = 10.0;

    rclcpp::Service<robot::srv::RobotControl>::SharedPtr control_service;
    rclcpp::Publisher<robot::msg::Motors>::SharedPtr motor_publisher;
    rclcpp::Publisher<robot::msg::Sensors>::SharedPtr sensor_publisher;

    void calculateMotorSpeeds(const ReqPtr request, ResPtr response);
public:
    Processing();
};

#endif // OWN_PROCESSING_H
