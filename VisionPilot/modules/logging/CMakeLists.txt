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
if(DEFINED ONNXRUNTIME_ROOT)
        target_include_directories(logging PUBLIC ${ONNXRUNTIME_ROOT}/include)
endif()

target_link_libraries(logging
        PUBLIC
        ${OpenCV_LIBS}
)