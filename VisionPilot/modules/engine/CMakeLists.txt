cmake_minimum_required(VERSION 3.22.1)

# ─── ONNX Runtime ─────────────────────────────────────────────────────────────
# Pass via:  cmake -DONNXRUNTIME_ROOT=<path> ..
if(NOT DEFINED ONNXRUNTIME_ROOT OR ONNXRUNTIME_ROOT STREQUAL "")
    message(FATAL_ERROR
        "[engine] ONNXRUNTIME_ROOT is not set.\n"
        "  cmake -DONNXRUNTIME_ROOT=<path/to/onnxruntime> ..")
endif()

add_library(engine
    src/onnx_engine.cpp
)

target_include_directories(engine
    PUBLIC
        include
        ${ONNXRUNTIME_ROOT}/include
)

target_link_libraries(engine
    PUBLIC
        ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so
)

target_compile_features(engine PUBLIC cxx_std_17)
