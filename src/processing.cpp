#include "rclcpp/rclcpp.hpp"

#include "robot/constants.h"
#include "robot/motion_model.h"
#include "robot/processing.h"

#include "robot_interfaces/srv/robot_control.hpp"
#include "robot_interfaces/msg/motor_speeds.hpp"
#include "robot_interfaces/msg/sensors.hpp"
#include "robot_interfaces/msg/robot.hpp"

#include <string>

// using std::placeholders::_1;
// using std::placeholders::_2;

using ReqPtr = std::shared_ptr<robot_interfaces::srv::RobotControl::Request>;
using ResPtr = std::shared_ptr<robot_interfaces::srv::RobotControl::Response>;

Processing::Processing(const std::string kPropertiesFileName) :
    Node("processing_node")
{
    this->robot = RobotModel::createRobotModel(kPropertiesFileName);
    this->motion_model = std::make_shared<MotionModel>(this->robot);

    // Lambdas vs std::bind
    // https://answers.ros.org/question/299126
    // https://stackoverflow.com/a/17545183
    this->control_service =
        this->create_service<robot_interfaces::srv::RobotControl>("robot_control",
            [this](const ReqPtr req, ResPtr res) { this->model(req, res); });
            // std::bind(&Processing::calculateMotorSpeeds, this, _1, _2));

    this->robot_publisher =
        this->create_publisher<robot_interfaces::msg::Robot>(
            "robot/robot", Constants::QueueSize);

    this->motor_publisher =
        this->create_publisher<robot_interfaces::msg::MotorSpeeds>(
            "robot/motors", Constants::QueueSize);

    this->sensor_publisher =
        this->create_publisher<robot_interfaces::msg::Sensors>(
            "robot/sensors", Constants::QueueSize);

    this->pose_publisher =
        this->create_publisher<robot_interfaces::msg::Pose>(
            "robot/pose", Constants::QueueSize);
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

    this->motion_model->forwardKinematics(
        request->left_motor_encoder, request->right_motor_encoder);

    auto robot_message = robot_interfaces::msg::Robot();
    robot_message.pose = this->motion_model->getOdometryPose().toMsg();

    robot_message.sensors.sensor1 = request->sensor1;
    robot_message.sensors.sensor2 = request->sensor2;
    robot_message.sensors.sensor3 = request->sensor3;
    robot_message.sensors.sensor4 = request->sensor4;

    robot_message.speeds.left = speeds.left;
    robot_message.speeds.right = speeds.right;

    robot_message.encoders.left = request->left_motor_encoder;
    robot_message.encoders.right = request->right_motor_encoder;

    this->robot_publisher->publish(robot_message);
}

