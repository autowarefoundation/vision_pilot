cmake_minimum_required(VERSION 3.22.1)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_library(camera_subscriber
        src/ros2_to_opencv.cpp
)

target_include_directories(camera_subscriber
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(camera_subscriber
        PUBLIC
        rclcpp::rclcpp
        ${sensor_msgs_LIBRARIES}
        cv_bridge::cv_bridge
        ${OpenCV_LIBRARIES}
)

target_compile_features(camera_subscriber PUBLIC cxx_std_17)
