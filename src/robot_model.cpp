#include "robot/robot_model.h"

#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"

#include "robot/motors.h"
#include "robot/pose.h"
#include "robot/sensor.h"
#include "robot/speed_units.h"

////////////////////////////////////////////////////////////////////////////////
// Private functions

std::vector<Sensor> RobotModel::initialiseSensors(const std::string kFileName)
{
    std::ifstream pose_file(kFileName);

    std::vector<Sensor> sensors;

    double x, y, theta;
    char separator; // Unused

    int count = 0;

    while ((count++ < this->kNumberOfSensors) &&
           // https://stackoverflow.com/a/7868998
           (pose_file >> x >> separator >> y >> separator >> theta))
    {
        sensors.push_back({0.0, {x, y, theta}});
    }

    return sensors;
}

////////////////////////////////////////////////////////////////////////////////
// Constructor

RobotModel::RobotModel(const YAML::Node props) :
    kRobotRadius(props["general"]["robot_radius"].as<double>()),
    kWheelRadius(props["general"]["wheel_radius"].as<double>()),
    kAxleLength(props["general"]["axle_length"].as<double>()),
    kThreshold(props["general"]["threshold"].as<double>()),
    kNumberOfSensors(props["sensors"]["number"].as<int>()),
    kSpeedStraight(props["motors"]["speed"]["straight"].as<int>()),
    kSpeedTurning(props["motors"]["speed"]["turning"].as<int>()),
    pose({0.0,0.0,0.0})
{
    // Should this inherit from rclcpp::Node and have these declared as
    // parameters rather than constants?

    this->sensors = this->initialiseSensors(props["sensors"]["poses"].as<std::string>());
    this->motors = {0, 0};
}

////////////////////////////////////////////////////////////////////////////////
// Public functions

// Static
std::shared_ptr<RobotModel> RobotModel::createRobotModel(const std::string kFileName)
{
    const YAML::Node robot_props = YAML::LoadFile(kFileName);
    /// @TODO: Validate YAML
    return std::make_shared<RobotModel>(robot_props);
}

double RobotModel::getRadius()
{
    return this->kRobotRadius;
}

double RobotModel::getWheelRadius()
{
    return this ->kWheelRadius;
}

double RobotModel::getAxleLength()
{
    return this->kAxleLength;
}

Pose RobotModel::getPose()
{
    return this->pose;
}

void RobotModel::setPose(Pose pose)
{
    this->pose = pose;
}

int RobotModel::getNumberOfSensors()
{
    return this->kNumberOfSensors;
}


double RobotModel::getSensorValue(const int kSensorId)
{
    if (kSensorId >= 0 && kSensorId < this->kNumberOfSensors)
    {
        return this->sensors.at(kSensorId).reading;
    }
    return -1;
}

std::vector<double> RobotModel::getSensorValues()
{   
    std::vector<double> readings;
    for (Sensor sensor: this->sensors)
    {
        readings.push_back(sensor.reading);
    }

    return readings;
}

void RobotModel::setSensorValue(const int kSensorId, const double kValue)
{
    if (kSensorId >= 0 && kSensorId < this->kNumberOfSensors)
    {
        this->sensors.at(kSensorId).reading = kValue;
    }
}

double RobotModel::getMotorSpeed(const MotorSide kMotorSide, const SpeedUnit kUnitType)
{   
    const double kDegressToRadiansConversionFactor = 0.017453;

    int motor_speed = this->motors.left;
    if (kMotorSide == MotorSide::Right)
    {
        motor_speed = this->motors.right;
    }

    switch(kUnitType)
    {
        case SpeedUnit::DEG:
            return motor_speed;
        case SpeedUnit::RAD:
            return motor_speed * kDegressToRadiansConversionFactor;
        case SpeedUnit::CM:
            return ((motor_speed / 360.0) * (M_PI * 2.0 * this->kWheelRadius)) / 10.0;
        case SpeedUnit::MM:
            return (motor_speed / 360.0) * (M_PI * 2.0 * this->kWheelRadius);
        default:
            return 0.0;
    }
}

Motors RobotModel::getMotorSpeeds(const SpeedUnit kUnitType)
{
    return Motors {
        this->getMotorSpeed(MotorSide::Left, kUnitType),
        this->getMotorSpeed(MotorSide::Right, kUnitType)
    };
}

// Static
// int RobotModel::convertMotorSpeedToDPS(const double speed, const SpeedUnit unit_type)
// {
    
// }

void RobotModel::thresholdSpeedCalculator()
{
    Motors speeds = { this->kSpeedStraight, this->kSpeedStraight };
    if (this->sensors.at(0).reading < this->kThreshold ||
        this->sensors.at(1).reading < this->kThreshold)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turn left");
        speeds.left    = -(this->kSpeedTurning);
        speeds.right   = this->kSpeedTurning;
    }
    else if (this->sensors.at(2).reading < this->kThreshold ||
            this->sensors.at(3).reading < this->kThreshold)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turn right");
        speeds.left    = this->kSpeedTurning;
        speeds.right   = -(this->kSpeedTurning);
    }

    this->motors = speeds;
}