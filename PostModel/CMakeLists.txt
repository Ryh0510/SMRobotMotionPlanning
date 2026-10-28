#############################################################
#	CMake requirement (From Modern CMake)
#############################################################
cmake_minimum_required(VERSION 3.20)

#	Set project name
get_filename_component( TARGET_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME )
message( STATUS "\n-------------  Config ${PACKAGE_NAME}::${TARGET_NAME}: ${CMAKE_CURRENT_SOURCE_DIR}  -------------" )


#############################################################
#	Add the head and source files to the current project
#############################################################
file(GLOB SOURCES src/*.cpp     src/*.c     )
file(GLOB HEADERS src/*.h       src/*.hpp   )
file(GLOB INTERFACE_HEADERS include/${TARGET_NAME}/*.h include/${TARGET_NAME}/*.hpp )

cmake_path(SET abs_include_dir
    NORMALIZE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
message( STATUS "abs_include_dir is ${abs_include_dir}" )

#############################################################
#	Create target and Set the target prorperties
#############################################################
#	Create target
add_library( ${TARGET_NAME} SHARED ${SOURCES} ${HEADERS} ${INTERFACE_HEADERS} )
add_library( ${PACKAGE_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})
#  Set project include directories
target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
) 

#   Set Link Libraries
target_link_libraries ( ${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPublic}>
    PRIVATE
        $<BUILD_INTERFACE:${${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPrivate}>
)

#   Set the properties of the target
set_target_properties( ${TARGET_NAME} PROPERTIES 
    FOLDER                  ${PACKAGE_NAME}
    DEBUG_POSTFIX           _shared_${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX         _shared_${CMAKE_RELEASE_POSTFIX}
)

# target_compile_options( ${TARGET_NAME} 
#    PRIVATE
#         # $<$<CXX_COMPILER_ID:MSVC>:/bigobj>
#         -bigobj
#         # $<$<CXX_COMPILER_ID:GNU>:-Wa,-mbig-obj>
# )

# if( CMAKE_SYSTEM_NAME STREQUAL "Windows" )
#     target_compile_definitions( ${TARGET_NAME} 
#         PUBLIC
#             BOOST_USE_WINAPI_VERSION=BOOST_WINAPI_VERSION_WIN7
#     )
# else()
# endif()

#############################################################
#	Install Setting
#############################################################
if( ExportPacakge${PACKAGE_NAME}_${TARGET_NAME} )
    function_InstallTarget(
        TARGET_NAME ${TARGET_NAME}
        PACKAGE_NAME  ${PACKAGE_NAME}
        PACKAGE_INTERFACE_HEADERS ${abs_include_dir}
    )
endif()
# Set project compile definitions
target_compile_definitions( ${TARGET_NAME}
  PRIVATE 
   POSTINTERFACE_SHAREDLIB_EXPORTS
   POSTMODEL_SHAREDLIB_EXPORTS
    )
#############################################################
#	Runtime environment setting
#############################################################
#   Set runtime directory for VS project
if( CMAKE_SYSTEM_NAME MATCHES "Windows" )
set_property( TARGET ${TARGET_NAME} PROPERTY 
        VS_DEBUGGER_ENVIRONMENT $<$<CONFIG:Debug>:${VSDebugEnvPath}> )     
endif()

if(BuildTests)
    rs2026_add_test_unit(regression/PostProgramRunnerTest)
endif()


message( STATUS "-------------  Config ${PACKAGE_NAME}::${TARGET_NAME}: ${CMAKE_CURRENT_SOURCE_DIR}  -------------\n" )

