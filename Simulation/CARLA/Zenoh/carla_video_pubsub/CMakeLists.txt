cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(video_pubsub)

# Don't remove RPATH while install binaries
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

## Find libraries
find_package(OpenCV REQUIRED HINTS ${OPENCV_INSTALL_ROOT}/build/ QUIET)
find_package(zenohc REQUIRED)

string(APPEND CMAKE_CXX_FLAGS " -Wall -Wextra")
string(APPEND CMAKE_C_FLAGS " -Wall -Wextra")

## ONNX Runtime Providers
option(LIBPNG_ROOTDIR "libpng root dir")
set(CMAKE_CXX_STANDARD 17)

# set the MODEL_ROOTDIR variable to the path where the model is located
set(MODEL_ROOTDIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../VisionPilot/Middleware_Recipes/common")

include_directories(
                     ${OpenCV_INCLUDE_DIRS}    # OpenCV
                     ${MODEL_ROOTDIR}/include  # Common libraries
                   )

## Build & Link
add_executable(carla_video_subscriber carla_video_subscriber.cpp)
target_link_libraries(carla_video_subscriber PRIVATE ${OpenCV_LIBS} opencv_videoio opencv_imgproc opencv_highgui zenohc::lib)
add_executable(carla_segment_subscriber carla_segment_subscriber.cpp
	       ${MODEL_ROOTDIR}/visualizers/masks_visualization_engine.cpp)
target_link_libraries(carla_segment_subscriber PRIVATE ${OpenCV_LIBS} opencv_videoio opencv_imgproc opencv_highgui zenohc::lib)
add_executable(carla_depth_subscriber carla_depth_subscriber.cpp)
target_link_libraries(carla_depth_subscriber PRIVATE ${OpenCV_LIBS} opencv_videoio opencv_imgproc opencv_highgui zenohc::lib)


# Install
install(TARGETS carla_video_subscriber DESTINATION bin)
install(TARGETS carla_segment_subscriber DESTINATION bin)
install(TARGETS carla_depth_subscriber DESTINATION bin)
