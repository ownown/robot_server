#include "rclcpp/rclcpp.hpp"
#include "robot/srv/robot_control.hpp"

#include <memory>

void calculateMotorSpeeds(
    const std::shared_ptr<robot::srv::RobotControl::Request> request,
    std::shared_ptr<robot::srv::RobotControl::Response> response);

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("robot_server");

    rclcpp::Service<robot::srv::RobotControl>::SharedPtr service = 
        node->create_service<robot::srv::RobotControl>(
            "robot_control", &calculateMotorSpeeds);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server ready");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

void calculateMotorSpeeds(
    const std::shared_ptr<robot::srv::RobotControl::Request> request,
    std::shared_ptr<robot::srv::RobotControl::Response> response)
{
    const float kThreshold = 30.0;
    if (request->sensor1 < kThreshold || request->sensor2 < kThreshold)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turn left");
        response->left_motor = -330;
        response->right_motor = 330;
    }
    else if (request->sensor3 < kThreshold || request->sensor4 < kThreshold)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turn right");
        response->left_motor = 330;
        response->right_motor = -330;
    }
    else
    {
        response->left_motor = 330;
        response->right_motor = 330;
    }
}