cmake_minimum_required(VERSION 3.22.1)

add_library(engine
    src/onnx_engine.cpp
)

target_include_directories(engine
    PUBLIC
        include
)

target_link_libraries(engine
    PUBLIC
        logging
        onnxruntime::onnxruntime
)

target_compile_features(engine PUBLIC cxx_std_17)
