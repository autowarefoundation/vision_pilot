# Resolves ONNXRUNTIME_ROOT, the prebuilt ONNX Runtime the engine module links.
#
#   -DONNXRUNTIME_ROOT=<path>            use that tree (CPU, CUDA or TensorRT build)
#   -DVISIONPILOT_FETCH_ONNXRUNTIME=ON   otherwise download the pinned CPU release
#
# The download is on by default only for the Python build, so `pip install`
# works from a clean machine; the app build keeps requiring an explicit root.

set(VISIONPILOT_ONNXRUNTIME_VERSION 1.26.0)

if(DEFINED ONNXRUNTIME_ROOT AND NOT ONNXRUNTIME_ROOT STREQUAL "")
    return()
endif()
if(DEFINED ENV{ONNXRUNTIME_ROOT} AND NOT "$ENV{ONNXRUNTIME_ROOT}" STREQUAL "")
    set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "ONNX Runtime install tree")
    return()
endif()
if(NOT VISIONPILOT_FETCH_ONNXRUNTIME)
    return()  # modules/engine reports the missing root
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(_ort_arch x64)
    set(_ort_sha256 1254da24fb389cf39dc0ff3451ab48301740ffbfcbaf646849df92f80ee92c57)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(_ort_arch aarch64)
    set(_ort_sha256 34ff1c2d0f12e2cf3d33a0c5f82e39792e1d581fbd6968fd7c30d173654be01a)
else()
    message(FATAL_ERROR "No prebuilt ONNX Runtime for ${CMAKE_SYSTEM_PROCESSOR}; set ONNXRUNTIME_ROOT")
endif()

include(FetchContent)
FetchContent_Declare(onnxruntime
        URL "https://github.com/microsoft/onnxruntime/releases/download/v${VISIONPILOT_ONNXRUNTIME_VERSION}/onnxruntime-linux-${_ort_arch}-${VISIONPILOT_ONNXRUNTIME_VERSION}.tgz"
        URL_HASH SHA256=${_ort_sha256}
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(onnxruntime)
set(ONNXRUNTIME_ROOT "${onnxruntime_SOURCE_DIR}" CACHE PATH "ONNX Runtime install tree" FORCE)
message(STATUS "[onnxruntime] Using ${VISIONPILOT_ONNXRUNTIME_VERSION} (${_ort_arch}) at ${ONNXRUNTIME_ROOT}")
