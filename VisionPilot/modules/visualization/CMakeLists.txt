cmake_minimum_required(VERSION 3.22.1)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(WEBCONN IMPORTED_TARGET
        gstreamer-1.0
        gstreamer-app-1.0
        gstreamer-sdp-1.0
        gstreamer-webrtc-1.0
        libsoup-2.4
        json-glib-1.0
)

if (WEBCONN_FOUND)
    list(APPEND VIS_SOURCES src/webrtc_stream.cpp)
else ()
    message(WARNING "ENABLE_WEBRTC=ON but GStreamer WebRTC packages not found; building without WebRTC")
endif ()


add_library(visualization
        ../common/include/common/types.hpp
        src/visual_interface.cpp
        src/visualization.cpp
        src/local_display.cpp
        ${VIS_SOURCES}
)

if (WEBCONN_FOUND)
    target_compile_definitions(visualization PUBLIC ENABLE_WEBRTC=1 PRIVATE GST_USE_UNSTABLE_API=1)
    target_link_libraries(visualization PUBLIC PkgConfig::WEBCONN)
endif ()

target_include_directories(visualization
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(visualization
        PUBLIC
        ${OpenCV_LIBRARIES}
        Threads::Threads
        models
        common
)
