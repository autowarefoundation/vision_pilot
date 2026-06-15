cmake_minimum_required(VERSION 3.8)
project(sodasim_gstreamer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

include_directories(include ${GSTREAMER_INCLUDE_DIRS})

add_library(image_to_gstreamer_node SHARED
  src/image_to_gstreamer_node.cpp
)
ament_target_dependencies(image_to_gstreamer_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  OpenCV
)
target_include_directories(image_to_gstreamer_node PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(image_to_gstreamer_node ${GSTREAMER_LIBRARIES})
target_compile_options(image_to_gstreamer_node PRIVATE ${GSTREAMER_CFLAGS_OTHER})

rclcpp_components_register_node(
  image_to_gstreamer_node
  PLUGIN "sodasim::gstreamer::ImageToGStreamerNode"
  EXECUTABLE image_to_gstreamer_node_exe
)

install(
  TARGETS image_to_gstreamer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
