cmake_minimum_required(VERSION 3.22.1)
project(libpandadbw LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB DBW_SOURCES "src/*.cpp")

add_library(pandabw STATIC ${DBW_SOURCES})

if(NOT TARGET pandac)
    set(LIBPANDA_BUILDING_FROM_DBW ON CACHE INTERNAL "Building local libpanda without its DBW subproject" FORCE)
    add_subdirectory(
        "${CMAKE_CURRENT_SOURCE_DIR}/libpanda"
        "${CMAKE_CURRENT_BINARY_DIR}/libpanda_build"
    )
endif()

if(NOT TARGET NMEAParserLib)
    add_subdirectory(
        "${CMAKE_CURRENT_SOURCE_DIR}/libpanda/NMEAParser/Software"
        "${CMAKE_CURRENT_BINARY_DIR}/libpanda_nmea_build"
    )
endif()

target_include_directories(pandabw PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/libpanda/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/libpanda/NMEAParser/Software/NMEAParserLib/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../.."
)

if(TARGET pandac)
    target_link_libraries(pandabw PUBLIC pandac NMEAParserLib)
else()
    find_library(PANDAC_LIBRARY NAMES pandac PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/libpanda/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/libpanda/build
        ${CMAKE_CURRENT_SOURCE_DIR}/libpanda/build/lib
        /usr/local/lib
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        NO_DEFAULT_PATH)

    if(PANDAC_LIBRARY)
        message(STATUS "Found external libpandac: ${PANDAC_LIBRARY}")
        target_link_libraries(pandabw PUBLIC ${PANDAC_LIBRARY} NMEAParserLib)
    else()
        message(FATAL_ERROR "pandac target not found and libpandac not found on the system. Configure from the root libpanda project or build/install libpandac before building DBW.")
    endif()
endif()

install(TARGETS pandabw
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

# ================== Main DBW Test===============================
add_executable(dbw_main main.cpp)
target_link_libraries(dbw_main PRIVATE pandabw)

if(PANDAC_LIBRARY)
    target_link_libraries(dbw_main PRIVATE ${PANDAC_LIBRARY})
endif()

target_include_directories(dbw_main PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../.."
)
# ===============================================================

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/"
    DESTINATION include
)
