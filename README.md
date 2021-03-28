# Autonomous Mobile Robot - ROS2 Nodes

This is a part of the Honours Year Project of my BSc Computer Science at the University of Liverpool.

Still very much a work in progress.

## Summary

### Title

Showcasing Lego-based Raspberry Pi Robots with Dexter - BrickPi3

### Brief

The aim of this project is a proof of concept project to demonstrate how a Lego Robot (based on the design used in the Lab 4 Robot Lab) can be controlled using a Raspberry Pi 3 platform using the EV3 sensors / actuators.

Currently the Lego EV3 Bricks (running at 300MHz) are limited in the processing power and sensor control they provide. However, the Dexter - BrickPi3 Starter Kit [1] is a Lego compatible platform that can connect to Lego actuators and sensors, without the need for the EV3 brick.

The aim of the project is to implement a Robot that can explore a physical environment. This will involve investigating the libraries required by the Dexter platform.

## Notes

- All examples for building and running assume an environment similar to my own - i.e. a Debian-based Linux setup with Firefox. To build and run on Windows/Mac, you're on your own.

## Requirements

_This may work on other setups, but this is what I've built it for and tested it on_

- Ubuntu 20.04
- ROS2 Foxy (C++ not Python)
- doxygen (for documentation)

## Documentation

_Not yet in place_

Code documentation is handled by [Doxygen](https://www.doxygen.nl/index.html) and can be generated and viewed from the root directory with

```bash
doxygen Doxyfile
firefox ./html/index.html
```

## Build and run

I have struggled to build ROS2 packages on the Raspberry Pi itself (I'm _sure_ I could at the start, but I can't any more), so I recommend cross compiling using the ROS [cross compile tool](https://github.com/ros-tooling/cross_compile).

```bash
cd /path/to/workspace
ros_cross_compile ./ --arch aarch64 --os ubuntu --rosdistro foxy --colcon-defaults ./colcon_cross_compile.yml
scp -r install_aarch64 user@rpi_hostname:/path/to/workspace
ssh user@rpi_hostname
cd /path/to/workspace
. install_aarch64/setup.bash
ros2 run robot controls
```
