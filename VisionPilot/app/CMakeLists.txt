add_executable(VisionPilot
        vision_pilot.cpp
)

set_target_properties(VisionPilot PROPERTIES
        INSTALL_RPATH "/usr/share/visionpilot/onnxruntime/lib"
        BUILD_WITH_INSTALL_RPATH TRUE
)

add_dependencies(VisionPilot generate_config)

if (ENABLE_ROS2_INTERFACE)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    target_link_libraries(VisionPilot PUBLIC rclcpp::rclcpp)
    target_link_libraries(VisionPilot PRIVATE camera_ros2_interface vehicle_ros2_interface)
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
    vp_debug         # Debug annotate_frame
)

# Suppress unresolved-symbol errors from system shared libs (gdal→curl, ICU from miniforge3)
# that are pulled in transitively by opencv_viz; we never use viz ourselves.
target_link_options(VisionPilot PRIVATE -Wl,--allow-shlib-undefined)

# Installation rules
install(TARGETS VisionPilot
        RUNTIME DESTINATION bin
)

set(VISIONPILOT_CONF_FILES
        ${CMAKE_SOURCE_DIR}/config/H.yaml
        ${CMAKE_BINARY_DIR}/config/homography_C_matrix.yaml
        ${CMAKE_SOURCE_DIR}/config/vision_pilot.conf
        ${CMAKE_SOURCE_DIR}/config/vision_pilot_test.conf
)

if(ENABLE_ROS2_INTERFACE)
    list(APPEND VISIONPILOT_CONF_FILES
            ${CMAKE_SOURCE_DIR}/config/vision_pilot_ros2.conf
    )
endif()

install(FILES ${VISIONPILOT_CONF_FILES}
        DESTINATION share/visionpilot/config/
)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/assets/icons/"
        DESTINATION "/usr/share/visionpilot/assets/icons"
)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/modules/models/weights/"
        DESTINATION "/usr/share/visionpilot/modules/models/weights"
        FILES_MATCHING
        PATTERN "*.onnx"
        PATTERN "trt_cache" EXCLUDE
)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/onnxruntime/"
        DESTINATION "/usr/share/visionpilot/onnxruntime"
)
