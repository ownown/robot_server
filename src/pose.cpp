#include "robot/pose.h"

#include <cmath>
#include <string>

#ifdef OWN_USE_FORMAT
#include <format>
#else
#include <stdio.h>
#endif

Pose::Pose(double x, double y, double theta): x(x), y(y)
{
    this->setTheta(theta);
}

void Pose::setTheta(double theta)
{
    bool is_neg = theta < 0.0;

    theta = std::abs(theta);

    while (theta >= (2.0 * M_PI))
    {
        theta -= 2.0 * M_PI;
    }

    if (is_neg)
    {
        theta *= -1;
    }

    this->theta = theta;
}

void Pose::setPosition(Pose p)
{
    this->x = p.getX();
    this->y = p.getY();
    this->setTheta(p.getTheta());
}

double Pose::getX()
{
    return this->x;
}

double Pose::getY()
{
    return this->y;
}

double Pose::getTheta()
{
    return this->theta;
}

std::string Pose::toString()
{
    #ifdef OWN_USE_FORMAT
    return std::format("({:.03f}, {:.03f}, {:.03f})", this->x, this->y, this->theta);
    #else
    // As far as I'm aware, before C++20, C++ has no built in string formatting
    // options, so if we're not using C++20 we'll do this in a C-style way.
    char buffer[40];
    const char * const format = "(%.03f, %.03f, %.03f)";
    sprintf(buffer, format, this->x, this->y, this->theta);
    return std::string(buffer);
    #endif
}

robot_interfaces::msg::Pose Pose::toMsg()
{
    robot_interfaces::msg::Pose pose = robot_interfaces::msg::Pose();
    pose.x = this->x;
    pose.y = this->y;
    pose.theta = this->theta;

    return pose;
}