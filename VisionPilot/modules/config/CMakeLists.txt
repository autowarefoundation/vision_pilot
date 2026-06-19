cmake_minimum_required(VERSION 3.22.1)

add_library(config
    src/vision_pilot_config.cpp
)

target_include_directories(config
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(config
    PUBLIC
    engine   # for visionpilot::engine::EngineConfig in VisionPilotConfig
    models
)

target_compile_features(config PUBLIC cxx_std_17)
