cmake_minimum_required(VERSION 3.22.1)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(vehicle_ros2_interface
        src/vehicle_ros2_interface.cpp
)

target_include_directories(vehicle_ros2_interface
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(vehicle_ros2_interface
        PUBLIC
        vehicle_interface
        rclcpp::rclcpp
        std_msgs::std_msgs__rosidl_generator_cpp
        nav_msgs::nav_msgs__rosidl_generator_cpp
        nav_msgs::nav_msgs__rosidl_typesupport_cpp
        geometry_msgs::geometry_msgs__rosidl_generator_cpp
        geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
)