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

if(USE_ROS1)
  set(ENABLE_TESTING ${CATKIN_ENABLE_TESTING})
elseif(USE_ROS2)
  set(ENABLE_TESTING ${BUILD_TESTING})
endif()
# ##############################################################################
# END - OPTIONS           ##
# ##############################################################################

# ##############################################################################
# DEPENDENCIES            ##
# ##############################################################################
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrDependencies.cmake)



if(${COMPILE_ROS1_MODULE} OR ${COMPILE_ROS2_MODULE})
  set(AMENT_BUILDTOOL TRUE)
else()
  set(AMENT_BUILDTOOL FALSE)
endif()
cnr_install_directories(
  ${AMENT_BUILDTOOL} CNR_INSTALL_INCLUDE_DIR CNR_INSTALL_LIB_DIR
  CNR_INSTALL_BIN_DIR CNR_INSTALL_SHARE_DIR)
cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE USE_ROS1 USE_ROS2)
# ##############################################################################
# END DEPENDANCIES        ##
# ##############################################################################

# ##############################################################################
# Build       ##
# ##############################################################################
include(cnr_param_core.cmake)

if(COMPILE_ROS1_MODULE)
  set(ROS1_MODULE 1)
else()
  set(ROS1_MODULE 0)
endif()

if(COMPILE_ROS2_MODULE)
  include(cnr_param_ros2.cmake)
  set(ROS2_MODULE 1)
else()
  set(ROS2_MODULE 0)
endif()

if(COMPILE_MAPPED_FILE_MODULE)
  include(cnr_param_mapped_file.cmake)
  set(MAPPED_FILE_MODULE 1)
else()
  set(MAPPED_FILE_MODULE 0)
endif()

## To track the compilation options
configure_file(
  "${PROJECT_SOURCE_DIR}/include/cnr_param/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/cnr_param/config.h" 
  @ONLY)

add_library(cnr_param SHARED src/cnr_param/cnr_param.cpp)
target_include_directories(cnr_param
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include ${CMAKE_CURRENT_BINARY_DIR}/include/cnr_param/>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(cnr_param
  PUBLIC
  $<$<BOOL:${ROS1_MODULE}>:ROS1_MODULE=1>
  $<$<BOOL:${ROS2_MODULE}>:ROS2_MODULE=1>
  $<$<BOOL:${MAPPED_FILE_MODULE}>:MAPPED_FILE_MODULE=1>
  $<$<NOT:${ROS1_MODULE}>:ROS1_MODULE=0>
  $<$<NOT:${ROS2_MODULE}>:ROS2_MODULE=0>
  $<$<NOT:${MAPPED_FILE_MODULE}>:MAPPED_FILE_MODULE=0>
  )

add_library(cnr_param SHARED src/cnr_param/cnr_param.cpp)
target_include_directories(cnr_param PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/include/cnr_param/")

if(COMPILE_ROS2_MODULE)
  target_compile_definitions(cnr_param PRIVATE ROS2_MODULE)
  target_link_libraries(cnr_param PUBLIC cnr_param::cnr_param_ros2)
endif()

if(COMPILE_MAPPED_FILE_MODULE)
  target_compile_definitions(cnr_param PRIVATE MAPPED_FILE_MODULE)
  target_link_libraries(cnr_param PUBLIC cnr_param::cnr_param_mapped_file cnr_param::cnr_param_server_utilities)
endif()

add_library(cnr_param::cnr_param ALIAS cnr_param)
# #############################################################################
# Testing  ##
# #############################################################################
if(ENABLE_TESTING)

  ### EXECUTABLE
  #add_executable(test_cnr_param
  #  ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cnr_param_mapped_file.cpp)
  #gtest_discover_tests(test_cnr_param)
  #  cnr_configure_gtest(test_cnr_param
  #    cnr_param::cnr_param
  #    ${CMAKE_CURRENT_SOURCE_DIR}/include include)

  #  target_compile_definitions(test_cnr_param PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")


  find_package(GTest REQUIRED)
  include(GoogleTest)
  include(CTest)

  add_executable(test_cnr_param test/test_cnr_param_mapped_file.cpp)
  target_link_libraries(test_cnr_param
    PUBLIC
    cnr_param::cnr_param
    GTest::gtest_main
  )
  target_compile_definitions(test_cnr_param PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")
  target_compile_options(test_cnr_param PUBLIC $<$<CONFIG:Release>:-Ofast >)
#  gtest_discover_tests(test_cnr_param)  # bug related to linking...
  add_test(NAME test_cnr_param
    COMMAND test_cnr_param)


endif()


# ##############################################################################
# END - Build ##
# ##############################################################################

# ##############################################################################
# ## Coverage Testing     ##
# ##############################################################################
# if( ENABLE_TESTING AND ENABLE_COVERAGE_TESTING AND NOT WIN32 ) if(USE_ROS)
# message(STATUS "The coverage testing is enabled") set(COVERAGE_EXCLUDES
# "*/${PROJECT_NAME}/test*") add_code_coverage( NAME coverage_report
# DEPENDENCIES ${PROJECT_NAME}_test ) else() ## NOTHING TO DO, SINCE GTEST
# ALREADY CREATES THE COVERAGE ## IF THE COMPILING OPTIONS ARE SET endif()
# endif()

# ##############################################################################
# INSTALL         ##
# ##############################################################################
list(APPEND HEADERS_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include "${CMAKE_CURRENT_BINARY_DIR}/include/cnr_param" )
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

if(COMPILE_ROS2_MODULE)
  ament_package()
endif()
