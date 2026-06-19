cmake_minimum_required(VERSION 3.22.1)

add_subdirectory(app)
add_subdirectory(control)
add_subdirectory(sensing/image_preprocessing)
add_subdirectory(safety_guardian/planning)

if(ENABLE_ROS2_INTERFACE)
    add_subdirectory(middleware_interfaces/ros2_interface/camera_subscriber)
    add_subdirectory(middleware_interfaces/ros2_interface/control_cmd_publisher)
    add_subdirectory(middleware_interfaces/ros2_interface/vehicle_state_subscriber)
endif()
