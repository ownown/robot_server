#include "rclcpp/rclcpp.hpp"

#include "robot/motion_model.h"
#include "robot/processing.h"

#include "robot/srv/robot_control.hpp"
#include "robot/msg/motor_speeds.hpp"
#include "robot/msg/sensors.hpp"

#include <string>

// using std::placeholders::_1;
// using std::placeholders::_2;

using ReqPtr = std::shared_ptr<robot::srv::RobotControl::Request>;
using ResPtr = std::shared_ptr<robot::srv::RobotControl::Response>;

Processing::Processing(const std::string kPropertiesFileName) : Node("processing_node")
{
    this->robot = RobotModel::createRobotModel(kPropertiesFileName);
    this->motion_model = std::make_shared<MotionModel>(this->robot);

    // Lambdas vs std::bind
    // https://answers.ros.org/question/299126
    // https://stackoverflow.com/a/17545183
    this->control_service =
        this->create_service<robot::srv::RobotControl>("robot_control",
            [this](const ReqPtr req, ResPtr res) { this->model(req, res); });
            // std::bind(&Processing::calculateMotorSpeeds, this, _1, _2));

    this->motor_publisher =
        this->create_publisher<robot::msg::MotorSpeeds>("robot/motors", 10);

    this->sensor_publisher =
        this->create_publisher<robot::msg::Sensors>("robot/sensors", 10);

    this->pose_publisher = 
        this->create_publisher<robot::msg::Pose>("robot/pose", 10);
}

void Processing::model(const ReqPtr request, ResPtr response)
{   
    this->robot->setSensorValue(0, request->sensor1);
    this->robot->setSensorValue(1, request->sensor2);
    this->robot->setSensorValue(2, request->sensor3);
    this->robot->setSensorValue(3, request->sensor4);
    this->robot->thresholdSpeedCalculator();

    Motors speeds = this->robot->getMotorSpeeds();

    response->left_motor_speed = speeds.left;
    response->right_motor_speed = speeds.right;

    this->motion_model->forwardKinematics(request->left_motor_encoder, request->right_motor_encoder);
    auto pose_message = this->motion_model->getOdometryPose().toMsg();

    auto sensor_message = robot::msg::Sensors();

    sensor_message.sensor1 = request->sensor1;
    sensor_message.sensor2 = request->sensor2;
    sensor_message.sensor3 = request->sensor3;
    sensor_message.sensor4 = request->sensor4;

    auto motor_message  = robot::msg::MotorSpeeds();

    motor_message.left = response->left_motor_speed;
    motor_message.right = response->right_motor_speed;

    this->sensor_publisher->publish(sensor_message);
    this->motor_publisher->publish(motor_message);
    this->pose_publisher->publish(pose_message);
}

