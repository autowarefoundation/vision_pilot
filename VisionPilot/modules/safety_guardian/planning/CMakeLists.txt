cmake_minimum_required(VERSION 3.22.1)

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

include(FetchContent)

FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 5.0.0
        GIT_SHALLOW TRUE
)

# Skip building tests/docs
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)

# Install vendored Eigen headers to a private path so packaging doesn't
# collide with the system libeigen3-dev package
set(INCLUDE_INSTALL_DIR "include/visionpilot/eigen3" CACHE STRING "" FORCE)

FetchContent_MakeAvailable(eigen)

add_library(planning
        src/planning.cpp
        src/longitudinal_planning.cpp
        src/lateral_planning.cpp

)

target_include_directories(planning
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE
        ${IPOPT_INCLUDE_DIRS}
)

target_compile_definitions(planning
        PRIVATE
        HAVE_CSTDDEF
)

find_library(
        CPPAD_LIBRARY
        NAMES cppad_lib
        REQUIRED
)

get_filename_component(
        CPPAD_LIBRARY_DIR
        "${CPPAD_LIBRARY}"
        DIRECTORY
)

set(
        CPPAD_LIBRARY_DIR
        "${CPPAD_LIBRARY_DIR}"
        CACHE PATH
        "Directory containing the CppAD runtime library"
)

target_link_libraries(planning
        PUBLIC
        common
        Eigen3::Eigen
        PRIVATE
        ${IPOPT_LIBRARIES}
        ${CPPAD_LIBRARY}
)
