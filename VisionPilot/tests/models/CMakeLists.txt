cmake_minimum_required(VERSION 3.22.1)

find_package(GTest REQUIRED)

add_executable(models_tests
        test_tensor_prep.cpp
)

target_link_libraries(models_tests
        PRIVATE
            models
            GTest::gtest
            GTest::gtest_main
)

target_compile_features(models_tests PRIVATE cxx_std_17)
