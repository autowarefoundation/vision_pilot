find_package(OpenCV REQUIRED)

add_library(common
        src/utils.cpp
)

target_include_directories(common
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(common
        PUBLIC
        ${OpenCV_LIBS}
)