cmake_minimum_required(VERSION 3.22.1)

find_package(GTest REQUIRED)

add_executable(models_tests
        test_merged_contract.cpp
        test_detections.cpp
        test_renesas_artifacts.cpp
        test_merged_backend.cpp
        test_inference.cpp
        test_tensor_prep.cpp
)

target_link_libraries(models_tests
        PRIVATE
            models
            engine
            GTest::gtest
            GTest::gtest_main
)

target_compile_features(models_tests PRIVATE cxx_std_17)

option(ENABLE_MERGED_EQUIVALENCE_TEST
       "Build the split-vs-merged equivalence test (needs the ~147 MB merged ONNX)"
       OFF)

if(ENABLE_MERGED_EQUIVALENCE_TEST)
    target_sources(models_tests PRIVATE test_merged_equivalence.cpp)
endif()
