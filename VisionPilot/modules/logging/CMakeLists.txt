cmake_minimum_required(VERSION 3.22.1)
find_package(OpenCV REQUIRED)

add_library(logging
        src/logger.cpp
)

target_include_directories(logging
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(logging
        PUBLIC
        ${CMAKE_SOURCE_DIR}/modules/models/include
        ${CMAKE_SOURCE_DIR}/modules/common/include
)
target_include_directories(logging
        PUBLIC
        ${CMAKE_SOURCE_DIR}/modules/safety_guardian/fusion/include
)
target_include_directories(logging
        PUBLIC
        ${CMAKE_SOURCE_DIR}/modules/engine/include
)

target_link_libraries(logging
        PUBLIC
        ${OpenCV_LIBS}
        onnxruntime::onnxruntime
)


if(ENABLE_RERUN)
        if(DEFINED ENV{RERUN_SDK_ROOT} AND NOT DEFINED RERUN_SDK_ROOT)
                set(RERUN_SDK_ROOT $ENV{RERUN_SDK_ROOT} CACHE PATH "Path to rerun_cpp_sdk")
        endif()

        if(RERUN_SDK_ROOT AND EXISTS "${RERUN_SDK_ROOT}/src/rerun.hpp")
                message(STATUS "[logging] Using Rerun C++ SDK at ${RERUN_SDK_ROOT}")
                add_subdirectory(${RERUN_SDK_ROOT}
                                 ${CMAKE_BINARY_DIR}/rerun_sdk_build EXCLUDE_FROM_ALL)
        else()
                message(STATUS "[logging] Fetching Rerun C++ SDK 0.34.1 via FetchContent")
                include(FetchContent)
                FetchContent_Declare(rerun_sdk
                        URL https://github.com/rerun-io/rerun/releases/download/0.34.1/rerun_cpp_sdk.zip)
                FetchContent_MakeAvailable(rerun_sdk)
        endif()

        target_link_libraries(logging PRIVATE rerun_sdk)
        target_compile_definitions(logging PRIVATE ENABLE_RERUN)
endif()
