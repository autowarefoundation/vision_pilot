add_executable(VisionPilot
    vision_pilot.cpp
)

target_link_libraries(VisionPilot
    PRIVATE
    config           # VisionPilotConfig, load_vision_pilot_config
    logging          # VP_INFO / VP_WARN / VP_ERROR
    engine           # OnnxEngine
    models           # AutoDrive, AutoSteer, AutoSpeed
    fusion           # LongitudinalFusion (includes tracking internally)
    can_interface
    v4l2_interface
    camera_subscriber
    visualization
    rclcpp::rclcpp
)
