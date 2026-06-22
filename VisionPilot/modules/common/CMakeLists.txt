cmake_minimum_required(VERSION 3.22.1)

add_library(common INTERFACE)

target_include_directories(common
        INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)