find_package(OpenCV REQUIRED)

add_executable(test_vision_pilot
        test_vision_pilot.cpp
)

if (ENABLE_ROS2_INTERFACE)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    target_link_libraries(test_vision_pilot PUBLIC rclcpp::rclcpp)
    target_link_libraries(test_vision_pilot PRIVATE camera_subscriber vehicle_ros2_interface)
    target_compile_definitions(test_vision_pilot PRIVATE ENABLE_ROS2_INTERFACE)
endif ()

target_link_libraries(test_vision_pilot
        PRIVATE
        config           # VisionPilotConfig, load_vision_pilot_config
        logging          # VP_INFO / VP_WARN / VP_ERROR
        engine           # OnnxEngine
        models           # AutoDrive, AutoSteer, AutoSpeed
        fusion           # LongitudinalFusion, LateralFusion
        planning
        vp_debug         # annotate_frame, DebugView
        vehicle_interface
        camera_interface
        image_preprocessing
        visualization
)