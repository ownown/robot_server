cmake_minimum_required(VERSION 3.5)
project(robot_server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2)
find_package(robot_interfaces REQUIRED)

# Seems to be good enough
find_package(yaml-cpp REQUIRED)

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

target_link_libraries(server yaml-cpp)

ament_target_dependencies(server rclcpp robot_interfaces)
ament_target_dependencies(visualization rclcpp rcl visualization_msgs tf2 robot_interfaces)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(TARGETS
  # controls
  server
  visualization
  DESTINATION lib/${PROJECT_NAME})

ament_package()
