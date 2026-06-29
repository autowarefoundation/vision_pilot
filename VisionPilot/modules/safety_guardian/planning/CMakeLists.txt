cmake_minimum_required(VERSION 3.22.1)

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

# eigen-5.0.0
add_subdirectory(src/eigen-5.0.0 EXCLUDE_FROM_ALL)

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
        common
        Eigen3::Eigen
        PRIVATE
        ${IPOPT_LIBRARIES}
)