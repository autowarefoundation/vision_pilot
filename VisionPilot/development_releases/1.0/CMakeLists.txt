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

find_package(Python3 REQUIRED)

set(Python3_EXECUTABLE /usr/bin/python3.10 CACHE FILEPATH "" FORCE)
# Homography transformation
add_custom_target(generate_config ALL
        COMMAND Python3::Interpreter
        ${CMAKE_SOURCE_DIR}/scripts/find_homography_C_matrix.py
)

# Modules
add_subdirectory(modules/control)

add_subdirectory(modules/logging)

add_subdirectory(modules/sensing/image_preprocessing)

add_subdirectory(modules/middleware_interfaces/ros2_interface/camera_subscriber)
add_subdirectory(modules/middleware_interfaces/ros2_interface/control_cmd_publisher)
add_subdirectory(modules/middleware_interfaces/ros2_interface/vehicle_state_subscriber)
add_subdirectory(modules/middleware_interfaces/ros2_interface/visualization_publisher)

add_subdirectory(modules/models)

add_subdirectory(modules/safety_guardian/fusion)
add_subdirectory(modules/safety_guardian/planning)

add_subdirectory(modules/sensing/can_interface)
add_subdirectory(modules/sensing/v4l2_interface)

add_subdirectory(modules/visualization)

# Tests
add_subdirectory(tests)

# App
add_subdirectory(app)
