cmake_minimum_required(VERSION 3.22.1)

add_executable(VisionPilot_VisualizationTest
	test_visualization.cpp
)

target_compile_definitions(VisionPilot_VisualizationTest PRIVATE
	VISUALIZATION_TEST_ASSET_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_assets"
	VISUALIZATION_TEST_OUTPUT_PATH="${CMAKE_CURRENT_BINARY_DIR}/visualization_result.png"
)

target_link_libraries(VisionPilot_VisualizationTest
	PRIVATE
	visualization
)