find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(test_control_cmd_publisher
        test_control_cmd_publisher.cpp
)

target_link_libraries(test_control_cmd_publisher
        PRIVATE
        control_cmd_publisher
        control
        rclcpp::rclcpp
        ${ackermann_msgs_TARGETS}
)
