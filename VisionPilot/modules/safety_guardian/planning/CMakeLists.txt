cmake_minimum_required(VERSION 3.22.1)

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

# Modern CppAD (>= ~2023, both conda-forge and Debian/Ubuntu apt) moves
# CppAD::local::temp_file() out of the headers into a runtime shared library
# (libcppad_lib). Consumers of this target (tests, the app) must link it or the
# ADFun<>::Forward instantiation in lateral_planning.cpp fails to link. Optional so
# older header-only CppAD installs still build.
find_library(CPPAD_LIB NAMES cppad_lib)

# eigen-5.0.0
add_subdirectory(src/eigen-5.0.0)

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

target_link_libraries(planning
        PUBLIC
        Eigen3::Eigen
        $<$<BOOL:${CPPAD_LIB}>:${CPPAD_LIB}>
        PRIVATE
        ${IPOPT_LIBRARIES}
)
