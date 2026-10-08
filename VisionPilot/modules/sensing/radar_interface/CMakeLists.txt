cmake_minimum_required(VERSION 3.22.1)

add_library(radar_interface
        src/file_interface.cpp
)

target_include_directories(radar_interface
        PUBLIC
        include
)
target_link_libraries(radar_interface
        PUBLIC
        common
)
