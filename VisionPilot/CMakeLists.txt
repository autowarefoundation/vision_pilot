cmake_minimum_required(VERSION 3.22.1)

project(VisionPilot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Put all executables in build/
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


# ROS2 packages
option(ENABLE_ROS2_INTERFACE "Enable ROS2 support" OFF)

# OpenCV packages
#find_package(OpenCV REQUIRED)
#find_package(cv_bridge REQUIRED)


set(Python3_EXECUTABLE /usr/bin/python3.10 CACHE FILEPATH "" FORCE)
find_package(Python3 REQUIRED)

# Preprocess C: your full-frame ground H (YAML) + fixed VisionPilot model V in script
set(VISIONPILOT_GROUND_HOMOGRAPHY "" CACHE FILEPATH "Full-frame ground H YAML (pixel -> world)")

if(VISIONPILOT_GROUND_HOMOGRAPHY STREQUAL "")
    message(FATAL_ERROR
        "Set -DVISIONPILOT_GROUND_HOMOGRAPHY=<your_full_frame_H.yaml> when configuring.")
endif()

add_custom_target(generate_config ALL
        COMMAND Python3::Interpreter
        ${CMAKE_SOURCE_DIR}/scripts/find_homography_C_matrix.py
        --ground-h ${VISIONPILOT_GROUND_HOMOGRAPHY}
        --output ${CMAKE_BINARY_DIR}/config/homography_C_matrix.yaml
)

# Modules
add_subdirectory(modules/logging)

add_subdirectory(modules/config)

add_subdirectory(modules/control)

add_subdirectory(modules/sensing/image_preprocessing)

if(ENABLE_ROS2_INTERFACE)
    add_subdirectory(modules/middleware_interfaces/ros2_interface/camera_subscriber)
    add_subdirectory(modules/middleware_interfaces/ros2_interface/control_cmd_publisher)
    add_subdirectory(modules/middleware_interfaces/ros2_interface/vehicle_state_subscriber)
    add_subdirectory(modules/middleware_interfaces/ros2_interface/visualization_publisher)
endif()

add_subdirectory(modules/engine)
add_subdirectory(modules/models)

add_subdirectory(modules/safety_guardian/fusion)   # tracking is embedded inside fusion
add_subdirectory(modules/safety_guardian/planning)

add_subdirectory(modules/sensing/can_interface)
add_subdirectory(modules/sensing/camera_interface)

add_subdirectory(modules/visualization)

add_subdirectory(modules/debug)

# Tests
add_subdirectory(tests)

# App
add_subdirectory(app)
