/**
 * @file robot_model.h
 * @author Oliver Newman (oliver@olivernewman.co.uk)
 * @brief A class for modelling the robot
 * @version 0.1
 * @date 2021-04-17
 *
 * (c) Copyright 2021 Oliver Newman
 *
 */

#ifndef OWN_ROBOT_MODEL_H
#define OWN_ROBOT_MODEL_H

#include <memory>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "robot/pose.h"
#include "robot/speed_units.h"
#include "robot/motors.h"
#include "robot/sensor.h"

class RobotModel
{
private:
    const double kRobotRadius;  // Radius of the robot at its widest point
    const double kWheelRadius;  // Radius of the wheels of the robot
    const double kAxleLength;   // Length of the wheel axle to/from the centre
                                // of the wheels

    const double kThreshold;    // Threshold reading at which point the robot 
                                // should take action to avoid an obstacle

    const int kNumberOfSensors; // The number of active sensors on the robot

    const int kSpeedStraight;
    const int kSpeedTurning;

    Pose pose;
    std::vector<Sensor> sensors;
    Motors motors;

    /**
     * @brief Read in the sensor poses from a file and return them as a vector.
     *
     * There is no real checking in this function, if the file is not in the
     * correct format it won't be read (but I don't believe it will fail...)
     *
     * Each row should consist of three values, the x and y coordinates and the
     * theta angle (aka the yaw) of one pose. They should be separated by a
     * comma and no whitespace. There should be no header for the file.
     *
     * The function will read until either it reaches the value of
     * kNumberOfSensors or until it reaches a line that doesn't fit the format
     * <double><char><double><char><double>
     * be that the end of the file or just a malformed line
     *
     * x,y,theta
     * x,y,theta
     *
     * @param kFileName, the CSV file containing the sensor poses
     * @return std::vector<Pose>
     */
    std::vector<Sensor> initialiseSensors(const std::string kFileName);

public:
    /**
     * @brief Construct a new RobotModel. Should only be called within
     * createModelRobot().
     *
     * @param props
     */
    RobotModel(const YAML::Node props);
    /**
     * @brief Create a RobotModel. Use this instead of the constructor.
     *
     * By doing it this way, we can keep the attributes as const
     * @param file_name, the path to the yaml config file
     * @return RobotModel
     */
    static std::shared_ptr<RobotModel> createRobotModel(const std::string kFileName);
    // RobotModel(const double robot_radius, const double wheel_radius, const double axle_length);
    
    double getRadius();
    double getWheelRadius();
    double getAxleLength();

    Pose getPose();
    void setPose(Pose pose);
    int getNumberOfSensors();
    double getSensorValue(const int kSensorId);
    std::vector<double> getSensorValues();
    void setSensorValue(const int kSensorId, const double kValue);
    double getMotorSpeed(const MotorSide kMotorSide, const SpeedUnit kUnitType = SpeedUnit::DEG);
    Motors getMotorSpeeds(const SpeedUnit kUnitType = SpeedUnit::DEG);
    void setMotorSpeed(const MotorSide kMotorSide, const double kValue, const SpeedUnit kUnitType = SpeedUnit::DEG);
    // static int convertMotorSpeedToDPS(const double kSpeed, const SpeedUnit kUnitType = SpeedUnit::DEG);
    void thresholdSpeedCalculator();
};

#endif // OWN_ROBOT_MODEL_H
