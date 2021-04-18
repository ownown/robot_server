#include "robot/visualization.h"

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rcl/time.h"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"


#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" // tf2::toMsg()

#include "robot/constants.h"

using namespace std::chrono_literals;

using Marker = visualization_msgs::msg::Marker;
using MsgPose = geometry_msgs::msg::Pose;

////////////////////////////////////////////////////////////////////////////////
// Private functions

Marker Visualization::initialiseCylinderMarker()
{
    const uint32_t shape = Marker::CYLINDER;
    Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = this->clock.now();
    marker.ns = "test_shape";
    marker.id = 0;

    marker.type = shape;
    marker.action = Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration(1s);

    return marker;
}

void Visualization::robotCallback(const robot::msg::Robot::SharedPtr msg) const
{
    // geometry_msgs::msg::Pose consists of
    // Point pose (x,y,z) and
    // Rotation rotation (x,y,z,w)
    MsgPose pose;

    pose.position.x = msg->pose.x;
    pose.position.y = msg->pose.y;
    pose.position.z = 0.0;

    // Create quaternion from Roll Pitch Yaw, where Yaw == Theta
    tf2::Quaternion quat;
    quat.setRPY(0, 0, msg->pose.theta);
    pose.orientation = tf2::toMsg(quat);

    this->pose_publisher->publish(pose);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor

Visualization::Visualization() : Node("visualization"), clock(RCL_SYSTEM_TIME)
{
    this->marker_publisher =
        this->create_publisher<Marker>("viz/marker", Constants::QueueSize);
    this->pose_publisher =
        this->create_publisher<MsgPose>("viz/robot_pose", Constants::QueueSize);
    this->robot_subscription =
        this->create_subscription<robot::msg::Robot>(
            "robot/robot", Constants::QueueSize,
            [this](const robot::msg::Robot::SharedPtr msg) {
                this->robotCallback(msg); });
}

////////////////////////////////////////////////////////////////////////////////
// Public functions

