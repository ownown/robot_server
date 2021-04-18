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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robot/srv/robot_control.hpp"

#include "robot/msg/motor_speeds.hpp"
#include "robot/msg/sensors.hpp"
#include "robot/msg/robot.hpp"

#include "robot/robot_model.h"
#include "robot/motion_model.h"
#include "robot/motors.h"

using ReqPtr = std::shared_ptr<robot::srv::RobotControl::Request>;
using ResPtr = std::shared_ptr<robot::srv::RobotControl::Response>;

class Processing : public rclcpp::Node
{
private:
    std::shared_ptr<RobotModel> robot;
    std::shared_ptr<MotionModel> motion_model;

    // const int kNormalSpeed  = 330;
    // const int kTurningSpeed = 110;
    const float kThreshold  = 10.0;

    rclcpp::Service<robot::srv::RobotControl>::SharedPtr control_service;
    rclcpp::Publisher<robot::msg::Robot>::SharedPtr robot_publisher;
    rclcpp::Publisher<robot::msg::MotorSpeeds>::SharedPtr motor_publisher;
    rclcpp::Publisher<robot::msg::Sensors>::SharedPtr sensor_publisher;
    rclcpp::Publisher<robot::msg::Pose>::SharedPtr pose_publisher;

    void model(const ReqPtr request, ResPtr response);
public:
    Processing(const std::string kPropertiesFileName);
};

#endif // OWN_PROCESSING_H
