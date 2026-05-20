cmake_minimum_required(VERSION 3.22.1)

project(VisionPilot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Put all executables in build/
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ROS2 packages
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# OpenCV packages
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Modules
add_subdirectory(modules/engine)

add_subdirectory(modules/logging)

add_subdirectory(modules/config)

add_subdirectory(modules/control)

add_subdirectory(modules/middleware_interfaces/ros2_interface/camera_subscriber)
add_subdirectory(modules/middleware_interfaces/ros2_interface/control_cmd_publisher)
add_subdirectory(modules/middleware_interfaces/ros2_interface/vehicle_state_subscriber)
add_subdirectory(modules/middleware_interfaces/ros2_interface/visualization_publisher)

add_subdirectory(modules/models)

add_subdirectory(modules/safety_guardian/fusion)   # tracking is embedded inside fusion
add_subdirectory(modules/safety_guardian/planning)

add_subdirectory(modules/sensing/can_interface)
add_subdirectory(modules/sensing/v4l2_interface)

add_subdirectory(modules/visualization)

# App
add_subdirectory(app)
