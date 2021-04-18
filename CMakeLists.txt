cmake_minimum_required(VERSION 3.5)
project(robot)

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
  # set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 20)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2)

# Seems to be good enough
find_package(yaml-cpp REQUIRED)

set(srv_files "srv/RobotControl.srv")
set(msg_files "msg/Pose.msg" "msg/Sensors.msg" "msg/MotorSpeeds.msg" "msg/MotorEncoders.msg" "msg/Robot.msg")

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files})

add_library(BrickPi3 STATIC src/BrickPi3.cpp src/bp_spi.cpp)
target_include_directories(BrickPi3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Build the RPI controller
add_executable(controls src/controls.cpp)
target_include_directories(controls PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Build the processing server
add_executable(server src/server.cpp src/robot_model.cpp src/motion_model.cpp src/pose.cpp src/processing.cpp)
target_include_directories(server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Build the visualization server to interface with RViz
add_executable(visualization src/visualization.cpp src/visualization_server.cpp)
target_include_directories(visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(controls BrickPi3)
target_link_libraries(server yaml-cpp)

ament_target_dependencies(controls rclcpp)
ament_target_dependencies(server rclcpp)
ament_target_dependencies(visualization rclcpp rcl visualization_msgs tf2)

ament_export_dependencies(rosidl_default_runtime)

rosidl_target_interfaces(controls ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(server ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(visualization ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  controls
  server
  visualization
  DESTINATION lib/${PROJECT_NAME})

ament_package()
