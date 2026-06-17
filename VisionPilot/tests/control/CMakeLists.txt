add_executable(test_control
        test_control.cpp
)

target_link_libraries(test_control
        PRIVATE
        control
)

add_executable(test_dbw_adapter
        test_dbw_adapter.cpp
)

target_link_libraries(test_dbw_adapter
        PRIVATE
        control
)

add_executable(test_closed_loop
        test_closed_loop.cpp
)

target_link_libraries(test_closed_loop
        PRIVATE
        control
        planning
)
