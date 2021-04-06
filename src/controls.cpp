/**
 * @file controls_main.cpp
 * @author Oliver Newman (oliver@olivernewman.co.uk)
 * @brief 
 * @version 0.1
 * @date 2021-03-29
 * 
 * (c) Copyright 2021 Oliver Newman
 * 
 */
#include <chrono> // chrono literals
#include <csignal> // exit signals
#include <cstdint>  // int types
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "robot/srv/robot_control.hpp"

#include "BrickPi3/BrickPi3.h"

using namespace std::chrono_literals;

typedef struct Sensors
{
    sensor_ultrasonic_t s1;
    sensor_ultrasonic_t s2;
    sensor_ultrasonic_t s3;
    sensor_ultrasonic_t s4;
} Sensors;

void exit_signal_handler(int signo);
void initialiseSensors(std::shared_ptr<BrickPi3> bp, Sensors *sensors);
void getSensorReadings(std::shared_ptr<BrickPi3> bp, Sensors *sensors);

int main(int argc, char **argv)
{
    const auto kServerWait      = 1s;
    const auto kEventLoopDelay  = 50ms;

    std::shared_ptr<BrickPi3> bp = std::make_shared<BrickPi3>();
    Sensors sensors;

    rclcpp::init(argc, argv);

    signal(SIGINT, exit_signal_handler);
    initialiseSensors(bp, &sensors);
    bp->reset_motor_encoder(PORT_A + PORT_B);

    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("robot_controller");

    rclcpp::Client<robot::srv::RobotControl>::SharedPtr client =
        node->create_client<robot::srv::RobotControl>("robot_control");

    while (!client->wait_for_service(kServerWait))
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the server...");
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server found...");
    while (rclcpp::ok())
    {
        
        auto request = std::make_shared<robot::srv::RobotControl::Request>();
        getSensorReadings(bp, &sensors);

        request->sensor1 = sensors.s1.cm;
        request->sensor2 = sensors.s2.cm;
        request->sensor3 = sensors.s3.cm;
        request->sensor4 = sensors.s4.cm;

        auto response = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, response) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Response recieved, setting motors to <%ddeg/s, %ddeg/s>",
                response.get()->left_motor, response.get()->right_motor);
            bp->set_motor_dps(PORT_B, response.get()->left_motor);
            bp->set_motor_dps(PORT_A, response.get()->right_motor);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call server");
        }

        rclcpp::sleep_for(kEventLoopDelay);
    }

    rclcpp::shutdown();
    
    bp->reset_all();
    return 0;
}

void exit_signal_handler(int signo)
{
    if(signo == SIGINT)
    {
        // This feels a little dirty, but works
        BrickPi3 bp;
        bp.reset_all();
        rclcpp::shutdown();
        exit(-2);
    }
}

void initialiseSensors(std::shared_ptr<BrickPi3> bp, Sensors *sensors)
{
    const auto kSensorCheckDelay = 20ms;

    int error = bp->detect();

    if (error)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Unable to detect BrickPi");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BrickPi detected");
    }

    bp->set_sensor_type(PORT_1, SENSOR_TYPE_EV3_ULTRASONIC_CM);
    bp->set_sensor_type(PORT_2, SENSOR_TYPE_EV3_ULTRASONIC_CM);
    bp->set_sensor_type(PORT_3, SENSOR_TYPE_EV3_ULTRASONIC_CM);
    bp->set_sensor_type(PORT_4, SENSOR_TYPE_EV3_ULTRASONIC_CM);

    error = 4;
    while (error == 4)
    {
        error = 0;
        if (bp->get_sensor(PORT_1, &(sensors->s1)))
        {
            error++;
        } 
        if (bp->get_sensor(PORT_2, &(sensors->s2)))
        {
            error++;
        }
        if (bp->get_sensor(PORT_3, &(sensors->s3)))
        {
            error++;
        }
        if (bp->get_sensor(PORT_4, &(sensors->s4)))
        {
            error++;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error: %d", error);
        if (error == 4) 
        {
            rclcpp::sleep_for(kSensorCheckDelay);
        }
    }
}

void getSensorReadings(std::shared_ptr<BrickPi3> bp, Sensors *sensors)
{
    bp->get_sensor(PORT_1, &(sensors->s1));
    bp->get_sensor(PORT_2, &(sensors->s2));
    bp->get_sensor(PORT_3, &(sensors->s3));
    bp->get_sensor(PORT_4, &(sensors->s4));
}
