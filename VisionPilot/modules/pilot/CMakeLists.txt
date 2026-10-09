cmake_minimum_required(VERSION 3.22.1)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# One iteration of the VisionPilot loop (preprocess → inference → fusion →
# planning) with no I/O, shared by the app and the Python bindings so a
# closed-loop simulation runs exactly the code that ships.
add_library(pilot
        src/pilot.cpp
)

target_include_directories(pilot
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(pilot
        PUBLIC
        common
        engine
        models
        fusion
        planning
        image_preprocessing
        opencv_core
)

target_compile_features(pilot PUBLIC cxx_std_17)
