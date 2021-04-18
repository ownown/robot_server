/**
 * @file pose.h
 * @author Oliver Newman (oliver@olivernewman.co.uk)
 * @brief Stores the pose (x,y, and angle) of the robot
 * @version 0.1
 * @date 2021-03-06
 *
 * This is based on the Java class from the second assignment of Dr Terry
 * Payne's COMP329 module, which itself was derived from the original RoboSim
 * code written for the module in 2016.
 *
 * (c) Copyright 2021 Oliver Newman
 *
 */

#ifndef OWN_POSE_H
#define OWN_POSE_H

#include <string>

#include "robot/msg/pose.hpp"

/**
 * @brief Stores the pose (x,y,theta) of the robot
 *
 */
class Pose
{
private:
    double x;
    double y;
    double theta;
public:
    /**
     * @brief Construct a new Pose object
     *
     * @param x the initial x coordinate (default 0)
     * @param y the initial y coordinate (default 0)
     * @param theta the initial angle (default 0)
     */
    Pose(double x = 0.0, double y = 0.0, double theta = 0.0);

    /**
     * @brief Change the pose angle
     *
     * @param theta the angle to change to
     */
    void setTheta(double theta);

    /**
     * @brief Set the pose to match another one
     *
     * @param p the pose to match
     */
    void setPosition(Pose p);

    /**
     * @brief Get the x coordinate
     *
     * @return the x coordinate
     */
    double getX();

    /**
     * @brief Get the y coordinate
     *
     * @return the y coordinate
     */
    double getY();

    /**
     * @brief Get the pose angle
     *
     * @return the angle
     */
    double getTheta();

    /**
     * @brief Convert the pose to a string representation
     *
     * @return a string representation of the pose
     */
    std::string toString();

    robot::msg::Pose toMsg();
};

#endif // OWN_POSE_H
