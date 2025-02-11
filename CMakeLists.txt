cmake_minimum_required(VERSION 3.16.0)

# ##############################################################################
# PRELIMINARIES                                                               ##
# ##############################################################################
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrMacros.cmake)
get_project_name("package.xml" extracted_name extracted_version)

project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)

cnr_set_flags()
# ##############################################################################
# END - PRELIMINARIES                                                         ##
# ##############################################################################

# ##############################################################################
# OPTIONS                                                                     ##
# ##############################################################################
option(BUILD_UNIT_TESTS "Build the unit tests" OFF)
option(BUILD_INTEGRATION_TESTS "Build the integration tests" OFF)

option(RETRIVE_DEPENDENCIES "Download and install dependencies" ON)

option(BUILD_AS_A_CATKIN_PACKAGE "Build the library as a CATKIN package" OFF)
option(COMPILE_ROS1_MODULE "ROS 1 INTEGRATION SUPPORT" OFF)
option(COMPILE_ROS2_MODULE "ROS 2 INTEGRATION SUPPORT" OFF)
option(COMPILE_MAPPED_FILE_MODULE "MAPPED FILE SUPPORT" ON)

if(BUILD_UNIT_TESTS OR BUILD_INTEGRATION_TESTS)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

if(BUILD_AS_A_CATKIN_PACKAGE)
  if(NOT "$ENV{ROS_VERSION}" STREQUAL "1")
    message(FATAL_ERROR
      "You selected CATKIN compilation option, but ROS environment is not sourced.")
  endif()
endif()

if(COMPILE_ROS1_MODULE)
  if(NOT "$ENV{ROS_VERSION}" STREQUAL "1")
    message(FATAL_ERROR
      "You selected COMPILE_ROS1_MODULE compilation option, but ROS environment is not sourced.")
  endif()
  set(ROS1_MODULE 1)
else()
  set(ROS1_MODULE 0)
endif()

if(COMPILE_ROS2_MODULE)
  if(NOT "$ENV{ROS_VERSION}" STREQUAL "2")
    message(FATAL_ERROR
      "You selected COMPILE_ROS1_MODULE compilation option, but ROS environment is not sourced.")
  endif()
  message(STATUS "The ROS 2 Module will be compiled")
  set(ROS2_MODULE 1)
else()
  set(ROS2_MODULE 0)
endif()

if(COMPILE_MAPPED_FILE_MODULE)
  message(STATUS "The Mapped File Module will be compiled")
  set(MAPPED_FILE_MODULE 1)
else()
  set(MAPPED_FILE_MODULE 0)
endif()

configure_file("${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h"
  @ONLY)

configure_file("${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}-compile-options.cmake.in"
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}-compile-options.cmake"
  @ONLY)
# ##############################################################################
# END - OPTIONS                                                               ##
# ##############################################################################

# ##############################################################################
# DEPENDENCIES                                                                ##
# ##############################################################################
if(RETRIVE_DEPENDENCIES)
  find_file(CNR_YAML_CONFIG_FOUND cnr_yamlConfig.cmake PATH_SUFFIXES share/cnr_yaml/cmake)
  if(NOT CNR_YAML_CONFIG_FOUND)
    message(STATUS "We did not find 'cnr_yaml' in the standard locations.. We will  dowonload it, compile and install under /usr/local")
    cnr_vcs_download_and_install(.repos /usr/local)
  endif()
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-dependencies.cmake)

list(APPEND DEPENDENCIES_TARGETS
  cnr_yaml::cnr_yaml
  Boost::system
  Boost::filesystem
  Boost::program_options
  Boost::iostreams
  Boost::regex
  Eigen3::Eigen
)

if(COMPILE_ROS2_MODULE)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS "${rclcpp_INCLUDE_DIRS}" "${rmw_INCLUDE_DIRS}" "${rosidl_runtime_c_INCLUDE_DIRS}" "${rcl_interfaces_INCLUDE_DIRS}")
  list(APPEND DEPENDENCIES_LINK_LIBRARIES "${rclcpp_LIBRARIES}")
endif()

if(BUILD_AS_A_CATKIN_PACKAGE)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS "${catkin_INCLUDE_DIRS}")
  list(APPEND DEPENDENCIES_LINK_LIBRARIES "${catkin_LIBRARIES}")

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    DEPENDS Boost
    CFG_EXTRAS ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-compile-options.cmake
  )
endif()
# ##############################################################################
# END DEPENDANCIES                                                            ##
# ##############################################################################

# ##############################################################################
# Build                                                                       ##
# ##############################################################################
# Core Library
set(SRC_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/core)
list(APPEND SRC ${SRC_CORE_DIR}/colors.cpp ${SRC_CORE_DIR}/filesystem.cpp ${SRC_CORE_DIR}/string.cpp)

# ROS 1
if(COMPILE_ROS1_MODULE)
  set(SRC_ROS1_MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/ros)
  list(APPEND SRC ${SRC_ROS1_MODULE_DIR}/param.cpp ${SRC_ROS1_MODULE_DIR}/param_dictionary.cpp)
endif()

# ROS 2
if(COMPILE_ROS2_MODULE)
  set(SRC_ROS2_MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/ros2)
  list(APPEND SRC
    ${SRC_ROS2_MODULE_DIR}/param_dictionary.cpp
    ${SRC_ROS2_MODULE_DIR}/param.cpp
    ${SRC_ROS2_MODULE_DIR}/param_retriever.cpp
    ${SRC_ROS2_MODULE_DIR}/yaml_formatter.cpp)
endif()

# MAPPED FILE
if(COMPILE_MAPPED_FILE_MODULE)
  set(SRC_MAPPED_FILE_MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/mapped_file)
  list(APPEND SRC
    ${SRC_MAPPED_FILE_MODULE_DIR}/interprocess.cpp
    ${SRC_MAPPED_FILE_MODULE_DIR}/yaml_manager.cpp
    ${SRC_MAPPED_FILE_MODULE_DIR}/args_parser.cpp
    ${SRC_MAPPED_FILE_MODULE_DIR}/yaml_parser.cpp)
endif()

# cnr_param
list(APPEND SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/cnr_param.cpp)

# ## add library
add_library(${PROJECT_NAME} SHARED ${SRC})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
# set PIC
set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES CMAKE_POSITION_INDEPENDENT_CODE ON
  INSTALL_RPATH_USE_LINK_PATH TRUE
  OUTPUT_NAME ${PROJECT_NAME})

list(APPEND INSTALL_INTERFACE_INCLUDE_DIRS "${DEPENDENCIES_INCLUDE_DIRS}")

target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PUBLIC "${DEPENDENCIES_INCLUDE_DIRS}")

# ## link
target_link_libraries(${PROJECT_NAME}
  PUBLIC "${DEPENDENCIES_TARGETS}"
  PUBLIC "${DEPENDENCIES_LINK_LIBRARIES}")

# ## definitions
target_compile_definitions(${PROJECT_NAME}
  PUBLIC ROS1_MODULE=${ROS1_MODULE} ROS2_MODULE=${ROS2_MODULE} MAPPED_FILE_MODULE=${MAPPED_FILE_MODULE})

add_library(cnr_param::cnr_param ALIAS ${PROJECT_NAME})
# ##############################################################################

if(ROS2_MODULE)
  # EXECECUTABLES ############################################################
  # ros2_yaml_converter
  add_executable(ros2_yaml_converter
    ${SRC_ROS2_MODULE_DIR}/ros2_yaml_converter.cpp)
  target_include_directories(
    ros2_yaml_converter
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PUBLIC "${DEPENDENCIES_INCLUDE_DIRS}")
  target_link_libraries(ros2_yaml_converter PUBLIC ${PROJECT_NAME})
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
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_link_libraries(cnr_param_server PUBLIC cnr_param::cnr_param)
  list(APPEND EXECUTABLE_TARGETS_LIST cnr_param_server)
  # cnr_param_server END #######################################################
endif()
# ##############################################################################
# END - Build                                                                 ##
# ##############################################################################


# ##############################################################################
# TESTING                                                                     ##
# ##############################################################################
if(BUILD_UNIT_TESTS OR BUILD_INTEGRATION_TESTS)
  include(cmake/coverage.cmake)
  add_coverage_target("*/test/*")

  include(cmake/tests.cmake)
endif()
# ##############################################################################
# END TESTING                                                                 ##
# ##############################################################################

# ##############################################################################
# CONFIGURE AND INSTALL DESTINATIONS                                          ##
# ##############################################################################
if(BUILD_AS_A_CATKIN_PACKAGE)
  set(CNR_PACKAGE_LIB_DESTINATION "${CATKIN_GLOBAL_LIB_DESTINATION}")
  set(CNR_PACKAGE_BIN_DESTINATION "${CATKIN_GLOBAL_LIB_DESTINATION}/${PROJECT_NAME}")
  set(CNR_PACKAGE_INCLUDE_DESTINATION "${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  set(CNR_PACKAGE_CONFIG_DESTINATION "${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}")
else()
  set(CNR_PACKAGE_LIB_DESTINATION "lib")
  set(CNR_PACKAGE_BIN_DESTINATION "bin")
  set(CNR_PACKAGE_INCLUDE_DESTINATION "include")
  set(CNR_PACKAGE_CONFIG_DESTINATION "share/${PROJECT_NAME}/cmake")
endif()
# ##############################################################################
# END - CONFIGURE AND INSTALL DESTINATIONS                                    ##
# ##############################################################################

# ##############################################################################
# CONFIGURE THE PACKAGE                                           ##
# ##############################################################################
if(NOT BUILD_AS_A_CATKIN_PACKAGE)
  cnr_cmake_package_file(${PROJECT_NAME}
    "${EXECUTABLE_TARGETS_LIST}")
endif()
# ##############################################################################
# END - CONFIGURE THE PACKAGE                                           ##
# ##############################################################################

# ##############################################################################
# INSTALL THE PACKAGE                                           ##
# ##############################################################################
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CNR_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

install(
  TARGETS ${PROJECT_NAME} "${EXECUTABLE_TARGETS_LIST}"
  EXPORT "${TARGETS_EXPORT_NAME}"
  ARCHIVE DESTINATION ${CNR_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CNR_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CNR_PACKAGE_BIN_DESTINATION})

if(NOT BUILD_AS_A_CATKIN_PACKAGE)
  # ------------------------------------------------------------------------------
  # Install cmake targets files and also namespaced alias targets
  install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${CONFIG_NAMESPACE}"
    DESTINATION "${CNR_PACKAGE_CONFIG_DESTINATION}"
    FILE ${TARGETS_EXPORT_NAME}.cmake)

  # ------------------------------------------------------------------------------
  # Install cmake config files
  install(FILES "${PROJECT_CONFIG_OUTPUT}" "${VERSION_CONFIG}"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-dependencies.cmake"
    DESTINATION "${CNR_PACKAGE_CONFIG_DESTINATION}")
endif()
# ##############################################################################
# END INSTALL                                                                 ##
# ##############################################################################
