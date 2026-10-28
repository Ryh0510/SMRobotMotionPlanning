cmake_minimum_required(VERSION 3.20)

get_filename_component(TARGET_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "\n-------------  Config ${PACKAGE_NAME}::${TARGET_NAME}: ${CMAKE_CURRENT_SOURCE_DIR}  -------------")

file(GLOB SOURCES src/*.cpp src/*.c)
file(GLOB HEADERS src/*.h src/*.hpp)
file(GLOB INTERFACE_HEADERS include/${TARGET_NAME}/*.h include/${TARGET_NAME}/*.hpp)

cmake_path(SET abs_include_dir NORMALIZE "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(${TARGET_NAME} STATIC ${SOURCES} ${HEADERS} ${INTERFACE_HEADERS})
add_library(${PACKAGE_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})
if(MSVC)
    # Eigen/JSON search lambdas exceed the normal COFF section count in VS2019.
    set_source_files_properties(src/ExternalTrajectoryEvaluation.cpp PROPERTIES COMPILE_OPTIONS /bigobj)
endif()

target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${PACKAGE_NAME}/include>
)

target_link_libraries(${TARGET_NAME}
    PUBLIC
        ${${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPublic}
    PRIVATE
        ${${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPrivate}
)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)

# NVRTC is loaded privately from <executable>/cuda by GpuBaffleSearch.
# A CUDA toolkit/link dependency is not required for runtime compilation.
set(SMROBOT_CUDA_RUNTIME_DIR "${PREBUILD_DIR}/cuda-nvrtc" CACHE PATH
    "Directory containing the optional NVIDIA NVRTC runtime DLLs and license")
if(WIN32 AND IS_DIRECTORY "${SMROBOT_CUDA_RUNTIME_DIR}")
    file(GLOB _smrobot_nvrtc_files
        "${SMROBOT_CUDA_RUNTIME_DIR}/nvrtc64_*.dll"
        "${SMROBOT_CUDA_RUNTIME_DIR}/nvrtc-builtins64_*.dll")
    list(FILTER _smrobot_nvrtc_files EXCLUDE REGEX "\\.alt\\.dll$")
    if(EXISTS "${SMROBOT_CUDA_RUNTIME_DIR}/License.txt")
        list(APPEND _smrobot_nvrtc_files "${SMROBOT_CUDA_RUNTIME_DIR}/License.txt")
    endif()
    if(_smrobot_nvrtc_files)
        add_custom_target(ProjectMotionPlanningGpuRuntime
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/cuda"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${_smrobot_nvrtc_files} "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/cuda"
            COMMENT "Deploy optional NVIDIA NVRTC runtime"
            VERBATIM)
        add_dependencies(${TARGET_NAME} ProjectMotionPlanningGpuRuntime)
    endif()
endif()

add_executable(RwsBaffleInspect tools/RwsBaffleInspect.cpp)
target_link_libraries(RwsBaffleInspect PRIVATE ${TARGET_NAME} nlohmann_json::nlohmann_json)
target_compile_features(RwsBaffleInspect PRIVATE cxx_std_17)
copy_runtime_dlls(RwsBaffleInspect)

add_executable(RwsTrajectoryEvaluate tools/RwsTrajectoryEvaluate.cpp)
target_link_libraries(RwsTrajectoryEvaluate PRIVATE ${TARGET_NAME} nlohmann_json::nlohmann_json)
target_compile_features(RwsTrajectoryEvaluate PRIVATE cxx_std_17)
copy_runtime_dlls(RwsTrajectoryEvaluate)

add_executable(GaApfCdfCheck tools/GaApfCdfCheck.cpp)
target_link_libraries(GaApfCdfCheck PRIVATE ${TARGET_NAME} nlohmann_json::nlohmann_json)
target_compile_features(GaApfCdfCheck PRIVATE cxx_std_17)
if(MSVC)
    target_link_options(GaApfCdfCheck PRIVATE /DEBUG /MAP)
endif()
copy_runtime_dlls(GaApfCdfCheck)

add_executable(RwsTrajectorySegment tools/RwsTrajectorySegment.cpp)
target_link_libraries(RwsTrajectorySegment PRIVATE ${TARGET_NAME})
target_compile_features(RwsTrajectorySegment PRIVATE cxx_std_17)
copy_runtime_dlls(RwsTrajectorySegment)

option(BuildExternalTrajectoryTests "Build focused external trajectory and IK graph tests" OFF)

set_target_properties(${TARGET_NAME}
    PROPERTIES
        FOLDER ${PACKAGE_NAME}
        DEBUG_POSTFIX _static_${CMAKE_DEBUG_POSTFIX}
        RELEASE_POSTFIX _static_${CMAKE_RELEASE_POSTFIX}
)

if(ExportPacakge${PACKAGE_NAME}_${TARGET_NAME})
    function_InstallTarget(
        TARGET_NAME ${TARGET_NAME}
        PACKAGE_NAME ${PACKAGE_NAME}
        PACKAGE_INTERFACE_HEADERS ${abs_include_dir}
    )
endif()

rs_any_example_category_enabled(_build_examples ${PACKAGE_NAME})
if(_build_examples)
    rs_add_example_category_roots()
endif()

message(STATUS "-------------  Config ${PACKAGE_NAME}::${TARGET_NAME}  -------------\n")
