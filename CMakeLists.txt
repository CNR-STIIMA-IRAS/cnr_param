cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

# ##############################################################################
# PRELIMINARIES           ##
# ##############################################################################
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
option(ENABLE_TESTING "ENABLE TESTING" ON)
option(ENABLE_COVERAGE_TESTING "ENABLE COVERAGE TESTING" OFF)
option(COMPILE_EXAMPLE "COMPILE THE EXAMPLE" OFF)

if(${USER_BUILD_TOOL} STREQUAL "CATKIN")
  set(ENABLE_TESTING ${CATKIN_ENABLE_TESTING})
elseif(${USER_BUILD_TOOL} STREQUAL "AMENT")
  set(ENABLE_TESTING ${BUILD_TESTING})
endif()

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

cnr_install_directories(CNR_INSTALL_INCLUDE_DIR CNR_INSTALL_LIB_DIR
                        CNR_INSTALL_BIN_DIR CNR_INSTALL_SHARE_DIR)
cnr_enable_testing()
# ##############################################################################
# END DEPENDANCIES        ##
# ##############################################################################

# ##############################################################################
# Build       ##
# ##############################################################################
configure_file("${PROJECT_SOURCE_DIR}/include/cnr_param/config.h.in"
               "${CMAKE_CURRENT_BINARY_DIR}/include/cnr_param/config.h" @ONLY)

include(cnr_param_core.cmake)
if(ROS1_MODULE)
  include(cnr_param_ros1.cmake)
endif()
if(ROS2_MODULE)
  include(cnr_param_ros2.cmake)
endif()
if(MAPPED_FILE_MODULE)
  include(cnr_param_mapped_file.cmake)
endif()

add_library(cnr_param SHARED src/cnr_param/cnr_param.cpp)

target_include_directories(
  cnr_param
  PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include
         ${CMAKE_CURRENT_BINARY_DIR}/include/cnr_param/>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  cnr_param
  PUBLIC $<$<BOOL:${ROS2_MODULE}>:cnr_param::cnr_param_ros2>
         $<$<BOOL:${MAPPED_FILE_MODULE}>:cnr_param::cnr_param_mapped_file>
         $<$<BOOL:${MAPPED_FILE_MODULE}>:cnr_param::cnr_param_server_utilities>)

target_compile_definitions(
  cnr_param PUBLIC ROS1_MODULE=${ROS1_MODULE} ROS2_MODULE=${ROS2_MODULE}
                   MAPPED_FILE_MODULE=${MAPPED_FILE_MODULE})

add_library(cnr_param::cnr_param ALIAS cnr_param)
list(APPEND TARGETS_LIST cnr_param)
# ##############################################################################
# Testing  ##
# ##############################################################################
if(ENABLE_TESTING)
  add_executable(
    test_cnr_param
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cnr_param_mapped_file.cpp)

  gtest_discover_tests(test_cnr_param)

  cnr_configure_gtest(test_cnr_param cnr_param::cnr_param
                      ${CMAKE_CURRENT_SOURCE_DIR}/include include)

  target_compile_definitions(test_cnr_param
                             PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")
endif()
# ##############################################################################
# END - Build ##
# ##############################################################################

# ##############################################################################
# INSTALL         ##
# ##############################################################################
list(APPEND HEADERS_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     "${CMAKE_CURRENT_BINARY_DIR}/include/cnr_param")
if(USE_ROS1)
  cnr_install_catkin_policy()
else()
  set(PROJECT_VERSION 1.0.0)
  cnr_install_cmake_policy(
    ${CNR_INSTALL_INCLUDE_DIR}
    ${CNR_INSTALL_LIB_DIR}
    ${CNR_INSTALL_BIN_DIR}
    ${extracted_version}
    "${TARGETS_LIST}"
    ${HEADERS_DIRS}
    test/launch
    test/config)
endif()
# ##############################################################################
# END INSTALL     ##
# ##############################################################################

if(ROS2_MODULE)
  ament_package()
endif()
