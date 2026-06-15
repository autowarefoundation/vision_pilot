find_package(OpenCV REQUIRED)

add_executable(visualization_tests
        test_visualization.cpp
)

target_link_libraries(visualization_tests
        PRIVATE
            visualization
            ${OpenCV_LIBS}
)
