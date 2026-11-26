find_package(OpenCV REQUIRED)

add_executable(image_preprocessing_tests
        test_image_preprocessor.cpp
)

target_link_libraries(image_preprocessing_tests
        PRIVATE
            image_preprocessing
            ${OpenCV_LIBS}
)

if(VISIONPILOT_ENABLE_GUI)
    target_compile_definitions(
        image_preprocessing_tests
        PRIVATE
        VISIONPILOT_ENABLE_GUI
    )
endif()
