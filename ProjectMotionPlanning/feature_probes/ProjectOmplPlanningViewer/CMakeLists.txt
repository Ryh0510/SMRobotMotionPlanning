cmake_minimum_required(VERSION 3.20)

rs_example_category_enabled(_build_example FeatureProbes ${PACKAGE_NAME})
if(NOT _build_example)
    return()
endif()

set(EXAMPLE_TARGET_NAME ProjectOmplPlanningViewer)

add_executable(${EXAMPLE_TARGET_NAME} main.cpp)

target_include_directories(${EXAMPLE_TARGET_NAME}
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
)

target_link_libraries(${EXAMPLE_TARGET_NAME}
    PRIVATE
        ${PACKAGE_NAME}::ProjectMotionPlanning
        glfw::glfw
        Common::GLRuntime
        Common::CustomLog
        Common::Utility
        SMRobotCore::RobotRuntime
        SMRobotPlatform::CameraCore
        SMRobotPlatform::RenderCore
        SMRobotPlatform::SceneCore
        SMRobotPlatform::RobotRenderBridge
)

target_compile_features(${EXAMPLE_TARGET_NAME} PRIVATE cxx_std_17)

set_target_properties(${EXAMPLE_TARGET_NAME}
    PROPERTIES
        FOLDER "FeatureProbes/SMRobotMotionPlanning"
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

if(WIN32)
    target_link_libraries(${EXAMPLE_TARGET_NAME} PRIVATE user32)
    set_property(TARGET ${EXAMPLE_TARGET_NAME} PROPERTY
        VS_DEBUGGER_ENVIRONMENT $<$<CONFIG:Debug>:${VSDebugEnvPath}>)
endif()

copy_runtime_dlls(${EXAMPLE_TARGET_NAME})
