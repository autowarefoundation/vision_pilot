cmake_minimum_required(VERSION 3.22.1)

find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_library(control_cmd_publisher
        src/cmd_to_ros2.cpp
)

target_include_directories(control_cmd_publisher
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(control_cmd_publisher
        PUBLIC
        control
        rclcpp::rclcpp
        ${ackermann_msgs_TARGETS}
)

target_compile_features(control_cmd_publisher PUBLIC cxx_std_17)
