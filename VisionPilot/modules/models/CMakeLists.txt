cmake_minimum_required(VERSION 3.22.1)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(models
    src/auto_drive.cpp
    src/auto_steer.cpp
    src/auto_speed.cpp
    src/inference.cpp
)

target_include_directories(models
    PUBLIC
        include
)

# engine provides OnnxEngine + ORT headers/libs transitively
target_link_libraries(models
    PUBLIC
        common
        engine
        fusion
        logging
        opencv_core
        opencv_imgproc
)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/weights
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

target_compile_features(models PUBLIC cxx_std_17)
