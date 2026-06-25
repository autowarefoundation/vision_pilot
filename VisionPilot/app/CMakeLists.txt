add_executable(VisionPilot
    vision_pilot.cpp
)

add_dependencies(VisionPilot generate_config)

if (ENABLE_ROS2_INTERFACE)
    target_link_libraries(VisionPilot
            PRIVATE
            camera_subscriber
            vehicle_ros2_interface
    )

    target_compile_definitions(VisionPilot PRIVATE ENABLE_ROS2_INTERFACE)
endif ()

target_link_libraries(VisionPilot
    PRIVATE
    config           # VisionPilotConfig, load_vision_pilot_config
    logging          # VP_INFO / VP_WARN / VP_ERROR
    common           # Common module
    engine           # OnnxEngine
    models           # AutoDrive, AutoSteer, AutoSpeed
    fusion           # LongitudinalFusion, LateralFusion
    planning         # Planner
    vehicle_interface
    camera_interface
    image_preprocessing
    visualization
)

# Suppress unresolved-symbol errors from system shared libs (gdal→curl, ICU from miniforge3)
# that are pulled in transitively by opencv_viz; we never use viz ourselves.
target_link_options(VisionPilot PRIVATE -Wl,--allow-shlib-undefined)
