find_package(OpenCV REQUIRED)

add_executable(test_planning
        test_planning.cpp
)

if(CPPAD_LIBRARY_DIR)
    set_property(
            TARGET test_planning
            APPEND
            PROPERTY BUILD_RPATH "${CPPAD_LIBRARY_DIR}"
    )
endif()

option(
        VISIONPILOT_ENABLE_GUI
        "Enable OpenCV GUI output for the planner test"
        ON
)

if(VISIONPILOT_ENABLE_GUI)
    target_compile_definitions(
            test_planning
            PRIVATE
            VISIONPILOT_ENABLE_GUI
    )
endif()

#add_executable(test_longitudinal_planning
#        test_longitudinal_planning.cpp
#)
#
#add_executable(test_lateral_planning
#        test_lateral_planning.cpp
#)

target_link_libraries(test_planning
        PRIVATE
        planning
        ${OpenCV_LIBS}
)

#target_link_libraries(test_longitudinal_planning
#        PRIVATE
#            planning
#            ${OpenCV_LIBS}
#)
#
#target_link_libraries(test_lateral_planning
#        PRIVATE
#        planning
#        ${OpenCV_LIBS}
#)
