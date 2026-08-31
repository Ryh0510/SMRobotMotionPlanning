cmake_minimum_required(VERSION 3.20)

get_filename_component(TARGET_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "\n-------------  Config ${PACKAGE_NAME}::${TARGET_NAME}: ${CMAKE_CURRENT_SOURCE_DIR}  -------------")

file(GLOB SOURCES src/*.cpp src/*.c)
file(GLOB HEADERS src/*.h src/*.hpp)
file(GLOB INTERFACE_HEADERS include/${TARGET_NAME}/*.h include/${TARGET_NAME}/*.hpp)

cmake_path(SET abs_include_dir NORMALIZE "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(${TARGET_NAME} STATIC ${SOURCES} ${HEADERS} ${INTERFACE_HEADERS})
add_library(${PACKAGE_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})

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
