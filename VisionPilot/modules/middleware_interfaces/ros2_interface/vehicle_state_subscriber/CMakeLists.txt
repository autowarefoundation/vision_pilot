cmake_minimum_required(VERSION 3.22.1)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(vehicle_state_subscriber
    src/ros2_to_can.cpp
)

target_include_directories(vehicle_state_subscriber
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(vehicle_state_subscriber PUBLIC rclcpp std_msgs)

target_link_libraries(vehicle_state_subscriber
    PUBLIC
    rt
)

target_compile_features(vehicle_state_subscriber PUBLIC cxx_std_17)
