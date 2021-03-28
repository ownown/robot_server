cmake_minimum_required(VERSION 3.5)
project(robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

add_library(BrickPi3 STATIC src/BrickPi3.cpp src/bp_spi.cpp)
target_include_directories(BrickPi3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Build the RPI controller
add_executable(controls src/controls_main.cpp src/sensors.cpp)
target_include_directories(controls PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Build the processor
add_executable(processing src/processing_main.cpp src/processing.cpp)
target_include_directories(processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


target_link_libraries(controls BrickPi3)
ament_target_dependencies(controls rclcpp robot_interfaces)
ament_target_dependencies(processing rclcpp robot_interfaces)


install(TARGETS 
  controls
  processing
  DESTINATION lib/${PROJECT_NAME})

ament_package()
