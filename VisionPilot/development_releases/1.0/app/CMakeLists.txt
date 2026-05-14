add_executable(VisionPilot vision_pilot.cpp)

add_dependencies(VisionPilot generate_config)

target_link_libraries(VisionPilot
        PRIVATE
        can_interface
        image_preprocessing
        v4l2_interface
        camera_subscriber
        models
        visualization
        rclcpp::rclcpp
)