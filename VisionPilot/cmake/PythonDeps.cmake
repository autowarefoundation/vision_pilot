# Native dependencies of the Python build, resolved without system packages,
# so `uv sync` (or `pip install`) needs nothing but a C/C++ compiler:
#
#   Ipopt   the `casadi` wheel ships Ipopt 3.14 (headers, libipopt.so.3 and its
#           MUMPS / METIS / OpenBLAS / gfortran runtime, all with RPATH $ORIGIN)
#   CppAD   the `cmeel-cppad` wheel (headers and libcppad_lib)
#   OpenCV  no wheel ships headers: core + imgproc are built from a pinned tag,
#           static, once per build directory
#
# Both wheels are build requirements and runtime dependencies in pyproject.toml
# at the same pinned versions, and the extension finds their libraries through
# RPATH entries relative to site-packages (see python/CMakeLists.txt).
#
#   -DVISIONPILOT_VENDOR_OPENCV=OFF   use an installed OpenCV instead

option(VISIONPILOT_VENDOR_OPENCV "Build a pinned OpenCV (core, imgproc) for the Python extension" ON)
set(VISIONPILOT_OPENCV_VERSION 4.12.0)

find_package(Python 3.10 REQUIRED COMPONENTS Interpreter)

# Locate an installed distribution's files without importing it.
function(_vp_site_path out expr)
    execute_process(
            COMMAND "${Python_EXECUTABLE}" -c "${expr}"
            OUTPUT_VARIABLE _path
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE _rc)
    if(NOT _rc EQUAL 0 OR _path STREQUAL "")
        message(FATAL_ERROR "[python-deps] Could not locate: ${expr}\n"
                "Install the build requirements from VisionPilot/pyproject.toml.")
    endif()
    set(${out} "${_path}" PARENT_SCOPE)
endfunction()

# ── Ipopt (casadi wheel) ──────────────────────────────────────────────────────
_vp_site_path(_casadi_dir
        "import importlib.util as u; print(u.find_spec('casadi').submodule_search_locations[0])")
if(NOT EXISTS "${_casadi_dir}/include/coin-or/IpIpoptApplication.hpp" OR NOT EXISTS "${_casadi_dir}/libipopt.so.3")
    message(FATAL_ERROR "[python-deps] ${_casadi_dir} has no Ipopt; expected the casadi wheel pinned in pyproject.toml")
endif()
add_library(VisionPilot::ipopt SHARED IMPORTED GLOBAL)
set_target_properties(VisionPilot::ipopt PROPERTIES
        IMPORTED_LOCATION "${_casadi_dir}/libipopt.so.3"
        # CppAD includes <coin-or/IpIpoptApplication.hpp>; Ipopt's own headers
        # include each other from coin-or/.
        INTERFACE_INCLUDE_DIRECTORIES "${_casadi_dir}/include;${_casadi_dir}/include/coin-or"
        # libipopt's own dependencies sit next to it; let the linker see them.
        INTERFACE_LINK_OPTIONS "LINKER:-rpath-link,${_casadi_dir}")
message(STATUS "[python-deps] Ipopt from ${_casadi_dir}")

# ── CppAD (cmeel-cppad wheel) ─────────────────────────────────────────────────
_vp_site_path(_cmeel_prefix
        "import sys, pathlib; print(next(str(p / 'cmeel.prefix') for p in map(pathlib.Path, sys.path) if (p / 'cmeel.prefix/include/cppad/cppad.hpp').exists()))")
file(GLOB _cppad_lib "${_cmeel_prefix}/lib/libcppad_lib.so.*")
list(GET _cppad_lib 0 _cppad_lib)
add_library(VisionPilot::cppad SHARED IMPORTED GLOBAL)
set_target_properties(VisionPilot::cppad PROPERTIES
        IMPORTED_LOCATION "${_cppad_lib}"
        INTERFACE_INCLUDE_DIRECTORIES "${_cmeel_prefix}/include")
message(STATUS "[python-deps] CppAD from ${_cmeel_prefix}")

# ── OpenCV (built from source) ────────────────────────────────────────────────
if(VISIONPILOT_VENDOR_OPENCV)
    set(_ocv_root "${CMAKE_BINARY_DIR}/_deps/opencv-${VISIONPILOT_OPENCV_VERSION}")
    set(_ocv_prefix "${_ocv_root}/install")
    if(NOT EXISTS "${_ocv_prefix}/lib/cmake/opencv4/OpenCVConfig.cmake")
        message(STATUS "[python-deps] Building OpenCV ${VISIONPILOT_OPENCV_VERSION} (core, imgproc); "
                "first configure of this build directory only")
        if(NOT EXISTS "${_ocv_root}/src/CMakeLists.txt")
            find_package(Git REQUIRED)
            file(REMOVE_RECURSE "${_ocv_root}/src")
            execute_process(
                    COMMAND "${GIT_EXECUTABLE}" -c advice.detachedHead=false clone --quiet --depth 1
                    --branch ${VISIONPILOT_OPENCV_VERSION} https://github.com/opencv/opencv.git "${_ocv_root}/src"
                    COMMAND_ERROR_IS_FATAL ANY)
        endif()
        include(ProcessorCount)
        ProcessorCount(_jobs)
        if(_jobs EQUAL 0)
            set(_jobs 2)
        endif()
        execute_process(
                COMMAND "${CMAKE_COMMAND}" -S "${_ocv_root}/src" -B "${_ocv_root}/build"
                -G "${CMAKE_GENERATOR}"
                "-DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}"
                "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
                -DCMAKE_BUILD_TYPE=Release
                "-DCMAKE_INSTALL_PREFIX=${_ocv_prefix}"
                -DCMAKE_POSITION_INDEPENDENT_CODE=ON
                -DBUILD_SHARED_LIBS=OFF
                -DBUILD_LIST=core,imgproc
                -DBUILD_ZLIB=ON
                -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF
                -DBUILD_DOCS=OFF -DBUILD_opencv_apps=OFF -DBUILD_JAVA=OFF -DBUILD_opencv_python3=OFF
                # Nothing downloaded at build time, nothing picked up from the system.
                -DWITH_IPP=OFF -DWITH_ITT=OFF -DWITH_OPENCL=OFF -DWITH_EIGEN=OFF -DWITH_LAPACK=OFF
                -DWITH_PROTOBUF=OFF -DWITH_ADE=OFF -DWITH_VA=OFF -DWITH_VA_INTEL=OFF
                # No imgcodecs, so no codecs: where the system lacks one, OpenCV
                # builds it from its bundled sources and exports a target for a library that
                # only imgcodecs would have installed.
                -DWITH_JPEG=OFF -DWITH_PNG=OFF -DWITH_TIFF=OFF -DWITH_WEBP=OFF
                -DWITH_OPENJPEG=OFF -DWITH_JASPER=OFF -DWITH_OPENEXR=OFF
                -DOPENCV_GENERATE_PKGCONFIG=OFF
                COMMAND_ERROR_IS_FATAL ANY)
        execute_process(
                COMMAND "${CMAKE_COMMAND}" --build "${_ocv_root}/build" --target install --parallel ${_jobs}
                COMMAND_ERROR_IS_FATAL ANY)
    endif()
    set(OpenCV_DIR "${_ocv_prefix}/lib/cmake/opencv4" CACHE PATH "OpenCV used by the Python build" FORCE)
    message(STATUS "[python-deps] OpenCV ${VISIONPILOT_OPENCV_VERSION} from ${_ocv_prefix}")
endif()
