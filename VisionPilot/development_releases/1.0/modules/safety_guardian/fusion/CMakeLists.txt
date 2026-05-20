cmake_minimum_required(VERSION 3.22.1)

find_package(OpenCV REQUIRED)

# tracking/ is a subfolder compiled directly into the fusion library (not a
# separate CMake target) so that tracking internals stay private to fusion.
add_library(fusion
    src/longitudinal_fusion.cpp
    src/lateral_fusion.cpp
    tracking/src/kalman_filter.cpp
    tracking/src/tracking_utils.cpp
    tracking/src/cipo_history.cpp
    tracking/src/feature_matching_utils.cpp
    tracking/src/object_finder.cpp
)

target_include_directories(fusion
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include          # <fusion/...>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tracking/include # <tracking/...> (internal only)
)

target_link_libraries(fusion
    PUBLIC
        models          # AutoDriveOutput, AutoSpeedOutput, Detection in public API
        ${OpenCV_LIBS}  # cv::Mat in public header
    PRIVATE
        logging         # VP_INFO macros used in longitudinal_fusion.cpp
)

target_compile_features(fusion PUBLIC cxx_std_17)
