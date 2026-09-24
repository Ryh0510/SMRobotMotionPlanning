cmake_minimum_required(VERSION 3.20)

rs_example_category_enabled(_build_example Regression ${PACKAGE_NAME})
if(NOT _build_example)
    return()
endif()

set(EXAMPLE_TARGET_NAME ProjectMotionPlanning-Headless)

add_executable(${EXAMPLE_TARGET_NAME} main.cpp)

target_include_directories(${EXAMPLE_TARGET_NAME}
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
)

target_link_libraries(${EXAMPLE_TARGET_NAME}
    PRIVATE
        ${PACKAGE_NAME}::ProjectMotionPlanning
        SMRobotCore::RobotRuntime
)

target_compile_features(${EXAMPLE_TARGET_NAME} PRIVATE cxx_std_17)

set_target_properties(${EXAMPLE_TARGET_NAME}
    PROPERTIES
        FOLDER "Regression/SMRobotMotionPlanning"
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

if(WIN32)
    set_property(TARGET ${EXAMPLE_TARGET_NAME} PROPERTY
        VS_DEBUGGER_ENVIRONMENT $<$<CONFIG:Debug>:${VSDebugEnvPath}>)
endif()

copy_runtime_dlls(${EXAMPLE_TARGET_NAME})

if(BuildTests)
    add_test(
        NAME ProjectMotionPlanningHeadless
        COMMAND $<TARGET_FILE:${EXAMPLE_TARGET_NAME}>
            --project ${CMAKE_SOURCE_DIR}/config/projects/420-red4600-tool.sys.json
    )
    set_tests_properties(ProjectMotionPlanningHeadless PROPERTIES
        LABELS "SMRobotMotionPlanning;ProjectMotionPlanning;Regression"
        WORKING_DIRECTORY $<TARGET_FILE_DIR:${EXAMPLE_TARGET_NAME}>
    )
    add_test(
        NAME ProjectMotionPlanningImportHeadless
        COMMAND $<TARGET_FILE:${EXAMPLE_TARGET_NAME}>
            --project ${CMAKE_SOURCE_DIR}/config/projects/420-red4600-tool.sys.json
            --import-only
    )
    set_tests_properties(ProjectMotionPlanningImportHeadless PROPERTIES
        LABELS "SMRobotMotionPlanning;ProjectMotionPlanning;Regression"
        WORKING_DIRECTORY $<TARGET_FILE_DIR:${EXAMPLE_TARGET_NAME}>
    )
endif()

if(BuildTests)
    add_executable(ProjectMotionPlanning-ApfTests ApfLocalPlannerTests.cpp)
    target_link_libraries(ProjectMotionPlanning-ApfTests PRIVATE ${PACKAGE_NAME}::ProjectMotionPlanning)
    target_compile_features(ProjectMotionPlanning-ApfTests PRIVATE cxx_std_17)
    set_target_properties(ProjectMotionPlanning-ApfTests PROPERTIES FOLDER "Regression/SMRobotMotionPlanning")
    copy_runtime_dlls(ProjectMotionPlanning-ApfTests)
    add_test(NAME ProjectMotionPlanningApf COMMAND $<TARGET_FILE:ProjectMotionPlanning-ApfTests>)
    set_tests_properties(ProjectMotionPlanningApf PROPERTIES
        LABELS "SMRobotMotionPlanning;ProjectMotionPlanning;Regression"
        WORKING_DIRECTORY $<TARGET_FILE_DIR:ProjectMotionPlanning-ApfTests>)
endif()

if(BuildTests)
    add_test(NAME ProjectMotionPlanningApfPeriodic
        COMMAND $<TARGET_FILE:ProjectMotionPlanning-Headless>
            --project ${CMAKE_SOURCE_DIR}/config/projects/ABB4600-burnner.sys.json
            --cdf-file ${CMAKE_CURRENT_SOURCE_DIR}/apf_periodic.txt --cdf-iterations 1)
    set_tests_properties(ProjectMotionPlanningApfPeriodic PROPERTIES
        LABELS "SMRobotMotionPlanning;ProjectMotionPlanning;Regression"
        WORKING_DIRECTORY $<TARGET_FILE_DIR:ProjectMotionPlanning-Headless>)
endif()

if(BuildTests)
    add_test(NAME ProjectMotionPlanningApfWrap
        COMMAND $<TARGET_FILE:ProjectMotionPlanning-Headless>
            --project ${CMAKE_SOURCE_DIR}/config/projects/ABB4600-burnner.sys.json
            --cdf-file ${CMAKE_CURRENT_SOURCE_DIR}/apf_wrap_boundary.txt
            --cdf-max-correction 0.05 --verification-step 0.001)
    set_tests_properties(ProjectMotionPlanningApfWrap PROPERTIES
        LABELS "SMRobotMotionPlanning;ProjectMotionPlanning;Regression"
        WORKING_DIRECTORY $<TARGET_FILE_DIR:ProjectMotionPlanning-Headless>)
endif()
