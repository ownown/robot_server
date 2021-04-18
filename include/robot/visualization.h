/**
 * @file visualization.h
 * @author Oliver Newman (oliver@olivernewman.co.uk)
 * @brief
 * @version 0.1
 * @date 2021-04-07
 *
 * (c) Copyright 2021 Oliver Newman
 *
 */
#ifndef OWN_VISUALIZATION_H
#define OWN_VISUALIZATION_H

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "robot_interfaces/msg/robot.hpp"

using Marker = visualization_msgs::msg::Marker;
using MsgPose = geometry_msgs::msg::PoseStamped;


class Visualization : public rclcpp::Node
{
private:
    rclcpp::Publisher<MsgPose>::SharedPtr pose_publisher;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr robot_subscription;
    rclcpp::Clock robot_clock;
    Marker robot_marker;

    Marker initialiseCylinderMarker();
    void robotCallback(const robot_interfaces::msg::Robot::SharedPtr msg);
public:
    Visualization();
    void moveRobotMarker();

};

#endif // OWN_VISUALIZATION_H
