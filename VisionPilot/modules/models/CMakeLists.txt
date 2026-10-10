cmake_minimum_required(VERSION 3.22.1)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp)
if(NOT NLOHMANN_JSON_INCLUDE_DIR)
    message(FATAL_ERROR
        "[models] nlohmann/json.hpp not found. Install nlohmann-json3-dev.")
endif()

add_library(models
    src/auto_drive.cpp
    src/auto_steer.cpp
    src/auto_speed.cpp
    src/backend.cpp
    src/split_backend.cpp
    src/inference.cpp
    src/tensor_prep.cpp
    src/merged_contract.cpp
    src/merged_backend.cpp
)

target_include_directories(models
    PUBLIC
        include
        ${NLOHMANN_JSON_INCLUDE_DIR}
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
