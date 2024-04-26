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
option(COMPILE_MAPPED_FILE_MODULE "MAPPED FILE SUPPORT" OFF)
option(ENABLE_TESTING "ENABLE TESTING" OFF)
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
include(cnr_param_utilities.cmake)

if(COMPILE_ROS2_MODULE)
  include(cnr_param_ros2.cmake)
endif()

if(COMPILE_MAPPED_FILE_MODULE)
  include(cnr_param_yaml_mapped_file.cmake)
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
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    test/launch
    test/config)
endif()
# ##############################################################################
# END INSTALL     ##
# ##############################################################################

if(COMPILE_ROS2_MODULE)
  ament_package()
endif()
