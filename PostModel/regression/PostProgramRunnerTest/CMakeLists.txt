cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME PostProgramRunnerTest)

add_executable(${TARGET_NAME} main.cpp)
target_link_libraries(${TARGET_NAME} PRIVATE SMRobotMotionPlanning::PostModel)
target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)

set_target_properties(${TARGET_NAME} PROPERTIES
    FOLDER SMRobotMotionPlanning/regression
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

add_test(NAME PostProgramRunner COMMAND ${TARGET_NAME})
