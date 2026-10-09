cmake_minimum_required(VERSION 3.22.1)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(fusion
    src/longitudinal_fusion.cpp
    src/radar_association.cpp
    src/lateral_fusion.cpp
)

target_include_directories(fusion
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include   # <fusion/...>
)

target_link_libraries(fusion
    PUBLIC
        models          # AutoDriveOutput, AutoSpeedOutput, Detection in public API
        opencv_core     # cv::Mat in public header
    PRIVATE
        logging         # VP_INFO macros
        opencv_imgproc  # perspectiveTransform in radar association
)

target_compile_features(fusion PUBLIC cxx_std_17)
