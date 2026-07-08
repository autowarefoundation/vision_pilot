cmake_minimum_required(VERSION 3.22.1)

project(camera_interface LANGUAGES CXX)

# Find required packages
find_package(OpenCV REQUIRED COMPONENTS core videoio)

add_library(camera_interface
        src/v4l2_camera_interface.cpp
        src/file_interface.cpp
)

# Set C++ standard
set_target_properties(camera_interface PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Include directories
target_include_directories(camera_interface
        PUBLIC
        include
)

# Link OpenCV libraries
target_link_libraries(camera_interface
        PUBLIC
        config
        logging
        opencv_core
        opencv_videoio
)

if(ENABLE_ROS2_INTERFACE)
    target_compile_definitions(camera_interface PRIVATE ENABLE_ROS2_INTERFACE)
    target_link_libraries(camera_interface PRIVATE camera_subscriber)
endif()

# Link threading support for std::mutex
find_package(Threads REQUIRED)
target_link_libraries(camera_interface
        PUBLIC
        Threads::Threads
)