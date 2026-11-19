cmake_minimum_required(VERSION 3.22.1)

option(
    VISIONPILOT_ENABLE_GUI
    "Enable OpenCV GUI output in tests"
    ON
)

add_subdirectory(sensing/image_preprocessing)
add_subdirectory(safety_guardian/planning)
