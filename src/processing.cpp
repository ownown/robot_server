#include "rclcpp/rclcpp.hpp"

#include "robot/processing.h"

#include "robot/srv/robot_control.hpp"
#include "robot/msg/motors.hpp"
#include "robot/msg/sensors.hpp"

// using std::placeholders::_1;
// using std::placeholders::_2;

using ReqPtr = std::shared_ptr<robot::srv::RobotControl::Request>;
using ResPtr = std::shared_ptr<robot::srv::RobotControl::Response>;

Processing::Processing() : Node("processing_node")
{
    // Lambdas vs std::bind
    // https://answers.ros.org/question/299126
    // https://stackoverflow.com/a/17545183
    this->control_service =
        this->create_service<robot::srv::RobotControl>("robot_control",
            [this](const ReqPtr req, ResPtr res) { this->calculateMotorSpeeds(req, res); });
            // std::bind(&Processing::calculateMotorSpeeds, this, _1, _2));

    this->motor_publisher =
        this->create_publisher<robot::msg::Motors>("robot/motors", 10);

    this->sensor_publisher =
        this->create_publisher<robot::msg::Sensors>("robot/sensors", 10);
}

void Processing::calculateMotorSpeeds(const ReqPtr request, ResPtr response)
{
    if (request->sensor1 < this->kThreshold ||
        request->sensor2 < this->kThreshold)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turn left");
        response->left_motor    = -(this->kTurningSpeed);
        response->right_motor   = this->kTurningSpeed;
    }
    else if (request->sensor3 < this->kThreshold ||
            request->sensor4 < this->kThreshold)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turn right");
        response->left_motor    = this->kTurningSpeed;
        response->right_motor   = -(this->kTurningSpeed);
    }
    else
    {
        response->left_motor    = this->kNormalSpeed;
        response->right_motor   = this->kNormalSpeed;
    }

    auto sensor_message = robot::msg::Sensors();

    sensor_message.sensor1 = request->sensor1;
    sensor_message.sensor2 = request->sensor2;
    sensor_message.sensor3 = request->sensor3;
    sensor_message.sensor4 = request->sensor4;

    auto motor_message  = robot::msg::Motors();

    motor_message.left_motor = response->left_motor;
    motor_message.right_motor = response->right_motor;

    this->sensor_publisher->publish(sensor_message);
    this->motor_publisher->publish(motor_message);
}
