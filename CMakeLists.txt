cmake_minimum_required(VERSION 3.16.0)

# ##############################################################################
# PRELIMINARIES           ##
# ##############################################################################

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrMacros.cmake)
get_project_name("package.xml" extracted_name extracted_version)

project(
  ${extracted_name}
  VERSION ${extracted_version}
  LANGUAGES CXX)

cnr_set_flags()
# ##############################################################################
# END - PRELIMINARIES     ##
# ##############################################################################

# ##############################################################################
# OPTIONS                 ##
# ##############################################################################
option(COMPILE_ROS1_MODULE "ROS 1 SUPPORT" OFF)
option(COMPILE_ROS2_MODULE "ROS 2 SUPPORT" OFF)
option(COMPILE_MAPPED_FILE_MODULE "MAPPED FILE SUPPORT" ON)
option(BUILD_UNIT_TESTS "Build the unit tests" OFF)
option(BUILD_INTEGRATION_TESTS "Build the integration tests" OFF)


if(COMPILE_ROS1_MODULE AND "$ENV{ROS_VERSION}" STREQUAL "1")
  message(STATUS "The ROS 1 Module will be compiled")
  set(ROS1_MODULE 1)
else()
  message(
    STATUS
      "The ROS 1 Module is not compiled (Running ROS version $ENV{ROS_VERSION})"
  )
  set(ROS1_MODULE 0)
endif()

if(COMPILE_ROS2_MODULE AND "$ENV{ROS_VERSION}" STREQUAL "2")
  message(STATUS "The ROS 2 Module will be compiled")
  set(ROS2_MODULE 1)
else()
  message(
    STATUS
      "The ROS 2 Module is not compiled (Running ROS version $ENV{ROS_VERSION})"
  )
  set(ROS2_MODULE 0)
endif()

if(COMPILE_MAPPED_FILE_MODULE)
  set(MAPPED_FILE_MODULE 1)
  message(STATUS "The Mapped File Module will be compiled")
else()
  set(MAPPED_FILE_MODULE 0)
  message(STATUS "The Mapped File Module won't be compiled")
endif()
# ##############################################################################
# END - OPTIONS           ##
# ##############################################################################

# ##############################################################################
# DEPENDENCIES            ##
# ##############################################################################
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrDependencies.cmake)
# ##############################################################################
# END DEPENDANCIES        ##
# ##############################################################################

# ##############################################################################
# Build       ##
# ##############################################################################
configure_file("${PROJECT_SOURCE_DIR}/include/cnr_param/config.h.in"
               "${PROJECT_SOURCE_DIR}/include/cnr_param/config.h" @ONLY)

# Core Library
set(SRC_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/core)
list(APPEND cnr_param_SRC ${SRC_CORE_DIR}/colors.cpp
     ${SRC_CORE_DIR}/filesystem.cpp ${SRC_CORE_DIR}/string.cpp)

# ROS 1
if(ROS1_MODULE)
  set(SRC_ROS1_MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/ros)
  list(APPEND cnr_param_SRC ${SRC_ROS1_MODULE_DIR}/param.cpp
       ${SRC_ROS1_MODULE_DIR}/param_dictionary.cpp)
endif()

# ROS 2
if(ROS2_MODULE)
  set(SRC_ROS2_MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/ros2)
  list(
    APPEND cnr_param_SRC ${SRC_ROS2_MODULE_DIR}/param_dictionary.cpp
    ${SRC_ROS2_MODULE_DIR}/param.cpp ${SRC_ROS2_MODULE_DIR}/param_retriever.cpp
    ${SRC_ROS2_MODULE_DIR}/yaml_formatter.cpp)
endif()

# MAPPED FILE
if(MAPPED_FILE_MODULE)
  set(SRC_MAPPED_FILE_MODULE_DIR
      ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/mapped_file)
  list(
    APPEND
    cnr_param_SRC
    ${SRC_MAPPED_FILE_MODULE_DIR}/interprocess.cpp
    ${SRC_MAPPED_FILE_MODULE_DIR}/yaml_manager.cpp
    ${SRC_MAPPED_FILE_MODULE_DIR}/args_parser.cpp
    ${SRC_MAPPED_FILE_MODULE_DIR}/yaml_parser.cpp)
endif()

# cnr_param
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME})
list(APPEND cnr_param_SRC ${SRC_DIR}/cnr_param.cpp)

# ## add library
add_library(cnr_param SHARED ${cnr_param_SRC})
add_library(cnr_param_static STATIC ${cnr_param_SRC})

# set PIC
set_target_properties(
  cnr_param
  PROPERTIES CMAKE_POSITION_INDEPENDENT_CODE ON
             INSTALL_RPATH_USE_LINK_PATH TRUE
             OUTPUT_NAME cnr_param)
set_target_properties(
  cnr_param_static PROPERTIES CMAKE_POSITION_INDEPENDENT_CODE ON OUTPUT_NAME
                                                                 cnr_param)

# Include DIRECTORIES: include and src
list(APPEND BUILD_INTERFACE_INCLUDE_DIRS "${DEPENDENCIES_INCLUDE_DIRS}"
     "${CMAKE_CURRENT_SOURCE_DIR}/include")
list(APPEND INSTALL_INTERFACE_INCLUDE_DIRS "${DEPENDENCIES_INCLUDE_DIRS}"
     include)

target_include_directories(
  cnr_param PUBLIC "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
                   "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>")
target_include_directories(
  cnr_param_static PUBLIC 
                    "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
                    "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>")

# ## link
target_link_libraries(
  cnr_param
  PUBLIC cnr_yaml::cnr_yaml
  PUBLIC "${DEPENDENCIES_LIBRARIES}")

target_link_libraries(
  cnr_param_static
  PUBLIC cnr_yaml::cnr_yaml_static
  PUBLIC "${DEPENDENCIES_LIBRARIES}")

# ## definitions
target_compile_definitions(
  cnr_param PUBLIC ROS1_MODULE=${ROS1_MODULE} ROS2_MODULE=${ROS2_MODULE}
                   MAPPED_FILE_MODULE=${MAPPED_FILE_MODULE})

# ## definitions
target_compile_definitions(
  cnr_param_static PUBLIC ROS1_MODULE=${ROS1_MODULE} ROS2_MODULE=${ROS2_MODULE}
                          MAPPED_FILE_MODULE=${MAPPED_FILE_MODULE})

add_library(cnr_param::cnr_param ALIAS cnr_param)
add_library(cnr_param::cnr_param_static ALIAS cnr_param_static)
list(APPEND LIBRARY_TARGETS_LIST cnr_param cnr_param_static)
# ##############################################################################

if(ROS2_MODULE)
  # EXECECUTABLES ############################################################
  # ros2_yaml_converter
  add_executable(ros2_yaml_converter
                 ${SRC_ROS2_MODULE_DIR}/ros2_yaml_converter.cpp)
  target_include_directories(
    ros2_yaml_converter
    PUBLIC "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
           "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>")
  target_link_libraries(ros2_yaml_converter PUBLIC cnr_param)
  set_target_properties(
    ros2_yaml_converter PROPERTIES LINK_FLAGS
                                   "-Wl,-rpath,${CNR_INSTALL_LIB_DIR}")
  list(APPEND EXECUTABLE_TARGETS_LIST ros2_yaml_converter)
endif()

if(MAPPED_FILE_MODULE)
  # cnr_param_server
  add_executable(cnr_param_server
                 ${SRC_MAPPED_FILE_MODULE_DIR}/param_server.cpp)

  target_include_directories(
    cnr_param_server
    PUBLIC "$<BUILD_INTERFACE:${MAPPED_FILE_BUILD_INTERFACE_INCLUDE_DIRS}>"
           "$<INSTALL_INTERFACE:${MAPPED_FILE_INSTALL_INTERFACE_INCLUDE_DIRS}>")

  target_link_libraries(cnr_param_server PUBLIC cnr_param::cnr_param)
  list(APPEND EXECUTABLE_TARGETS_LIST cnr_param_server)
  # cnr_param_server END #######################################################
endif()
# ##############################################################################
# END - Build ##
# ##############################################################################

# ##############################################################################
# INSTALL         ##
# ##############################################################################
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrInstall.cmake)

list(APPEND HEADERS_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     ${CMAKE_CURRENT_BINARY_DIR}/include)

cnr_install("${LIBRARY_TARGETS_LIST}" "${EXECUTABLE_TARGETS_LIST}"
            "${HEADERS_DIRS}")
# ##############################################################################
# END INSTALL     ##
# ##############################################################################

if(BUILD_UNIT_TESTS OR BUILD_INTEGRATION_TESTS)
  message(WARNING "Tests Added")
  add_subdirectory(test)
  message(WARNING "Converage target added")
  include(cmake/coverage.cmake)
  add_coverage_target("*/test/*")
endif()
