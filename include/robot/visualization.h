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
#include "visualization_msgs/msg/marker.hpp"

#include "robot/msg/robot.hpp"

using Marker = visualization_msgs::msg::Marker;


class Visualization : public rclcpp::Node
{
private:
    rclcpp::Publisher<Marker>::SharedPtr marker_publisher;
    rclcpp::Subscription<robot::msg::Robot>::SharedPtr robot_subscription;
    rclcpp::Clock clock;
    Marker robot_marker;
    Marker initialiseCylinderMarker();
    void robotCallback(const robot::msg::Robot::SharedPtr msg) const;
public:
    Visualization();
    void moveRobotMarker();

};

#endif // OWN_VISUALIZATION_H
