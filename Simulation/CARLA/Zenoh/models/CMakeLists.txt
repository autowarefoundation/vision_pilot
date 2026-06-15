cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(zenoh_models)

# Don't remove RPATH while install binaries
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(USE_CUDNN OFF)
set(CAFFE2_USE_CUDNN OFF)
set(USE_CUSPARSELT OFF)

if(USE_CUDA_BACKEND)
    add_definitions(-DUSE_EP_CUDA)
    message(STATUS "Using CUDA Execution Provider Backend")
elseif(USE_DNNL_BACKEND)
    add_definitions(-DUSE_EP_DNNL)
    message(STATUS "Using DNNL Execution Provider Backend")
endif()

## Find OpenCV installation
find_package(OpenCV REQUIRED HINTS ${OPENCV_INSTALL_ROOT}/build/ QUIET)
find_package(Torch REQUIRED HINTS ${LIBTORCH_INSTALL_ROOT}/share/cmake/Torch/ QUIET)

## Find Zenoh-C installation
find_package(zenohc REQUIRED)

## Enable CUDA if available (for GPU-accelerated mask creation)
find_package(CUDA QUIET)
if(CUDA_FOUND)
    enable_language(CUDA)
    add_definitions(-DCUDA_FOUND)
    message(STATUS "CUDA found. Enabling GPU acceleration for visualization.")
else()
    message(STATUS "CUDA not found. Using CPU-only visualization.")
endif()

string(APPEND CMAKE_CXX_FLAGS " -Wall -Wextra")
string(APPEND CMAKE_C_FLAGS " -Wall -Wextra")

## ONNX Runtime Configuration
option(onnxruntime_USE_CUDA "Build with CUDA support" OFF)
option(onnxruntime_USE_TENSORRT "Build with TensorRT support" OFF)
option(LIBPNG_ROOTDIR "libpng root dir")
option(ONNXRUNTIME_ROOTDIR "onnxruntime root dir")
set(CMAKE_CXX_STANDARD 17)

if(NOT ONNXRUNTIME_ROOTDIR)
    message(STATUS "You must provide an onnx runtime root directory!")
endif()

## TensorRT Configuration
find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS /usr/include/x86_64-linux-gnu/ /opt/tensorrt/include)
find_library(TENSORRT_LIBRARY nvinfer HINTS /usr/lib/x86_64-linux-gnu/)
find_library(TENSORRT_ONNXPARSER_LIBRARY nvonnxparser HINTS /usr/lib/x86_64-linux-gnu/)
if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY)
    message(FATAL_ERROR "Failed to find TensorRT.")
endif()

# set the MODEL_ROOTDIR variable to the path where the model is located
set(MODEL_ROOTDIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../VisionPilot/Middleware_Recipes/common")

# Set the include directories for ONNX Runtime, OpenCV, Torch, and common libraries
include_directories(${ONNXRUNTIME_ROOTDIR}/include
                    ${ONNXRUNTIME_ROOTDIR}/include/onnxruntime
                    ${ONNXRUNTIME_ROOTDIR}/include/onnxruntime/core/session
                    ${ONNXRUNTIME_ROOTDIR}/include/onnxruntime/core/providers/dnnl    # DNNL Backend Support
                    ${OpenCV_INCLUDE_DIRS}                                            # OpenCV
                    ${TORCH_INCLUDE_DIRS}                                             # Torch
                    ${MODEL_ROOTDIR}/include                                          # Common libraries
                   )

link_directories("${ONNXRUNTIME_ROOTDIR}/lib")

if(onnxruntime_USE_CUDA)
    add_definitions(-DUSE_CUDA)
endif()

## Build & Link
set(CARLA_RUN_MODEL_SOURCES
    carla_run_model.cpp
    # Existing segmentation/depth backends
    ${MODEL_ROOTDIR}/backends/tensorrt_backend.cpp
    ${MODEL_ROOTDIR}/backends/onnx_runtime_backend.cpp
    # Visualizers
    ${MODEL_ROOTDIR}/visualizers/masks_visualization_engine.cpp
    ${MODEL_ROOTDIR}/visualizers/depth_visualization_engine.cpp
    ${MODEL_ROOTDIR}/benchmark/fps_timer.cpp
)

if(CUDA_FOUND)
    # Add CUDA kernel for GPU-accelerated mask creation
    list(APPEND CARLA_RUN_MODEL_SOURCES ${MODEL_ROOTDIR}/visualizers/cuda_visualization_kernels.cu)
    set_property(SOURCE ${MODEL_ROOTDIR}/visualizers/cuda_visualization_kernels.cu PROPERTY LANGUAGE CUDA)
endif()

add_executable(carla_run_model ${CARLA_RUN_MODEL_SOURCES})
target_compile_definitions(carla_run_model PRIVATE LOG_TYPE=0)

if(CUDA_FOUND)
    target_include_directories(carla_run_model PRIVATE ${CUDA_INCLUDE_DIRS})
    set_target_properties(carla_run_model PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()

target_link_libraries(carla_run_model PRIVATE
                            onnxruntime
                            ${TENSORRT_LIBRARY}
                            ${TENSORRT_ONNXPARSER_LIBRARY}
                            ${TORCH_LIBRARIES}
                            ${OpenCV_LIBS}
                            opencv_videoio
                            opencv_imgproc
                            opencv_highgui
                            zenohc::lib
                     )

if(CUDA_FOUND)
    target_link_libraries(carla_run_model PRIVATE ${CUDA_LIBRARIES})
endif()

# Install
install(TARGETS carla_run_model DESTINATION bin)
