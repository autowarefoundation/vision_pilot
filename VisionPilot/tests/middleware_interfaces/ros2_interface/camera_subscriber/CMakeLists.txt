find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(test_camera_subscriber
        test_camera_subscriber.cpp
)

target_link_libraries(test_camera_subscriber
        PRIVATE
        camera_subscriber
        rclcpp::rclcpp
        ${sensor_msgs_TARGETS}
        ${OpenCV_LIBRARIES}
)
