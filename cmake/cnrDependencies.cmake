############################################
## It does takes inspiration from 
## Ton idea is to have a single file that can be included in the CMakeLists.txt of the package
## and in the >package>Config.cmake
## https://discourse.cmake.org/t/how-to-conditionally-call-either-find-pacakge-or-find-dependency/8175
include(CMakeFindDependencyMacro)

if(${PROJECT_NAME} STREQUAL "cnr_param")
  message(STATUS "Loading ''cnr_paramDependencies.cmake'' for the project '${PROJECT_NAME}'")
  macro(_find_package)
    find_package(${ARGN})
  endmacro()
else()
  message(STATUS "Running Project: ${PROJECT_NAME}. Loading cnr_paramDependencies.cmake from cnr_paramConfig.cmake")
  include(CMakeFindDependencyMacro)

  macro(_find_package)
    find_dependency(${ARGN})
  endmacro()
endif()

# ##########################################################################################
if(cnr_param_ROS1_MODULE OR ROS1_MODULE)
  message(STATUS "Find Catkin (REQUIRED) ...")
  _find_package(catkin REQUIRED COMPONENTS roscpp)
elseif(cnr_param_ROS2_MODULE OR ROS2_MODULE)
  message(STATUS "Find rclcpp (REQUIRED) ...")
  _find_package(rclcpp REQUIRED)
  _find_package(rmw REQUIRED)
  _find_package(rosidl_runtime_c REQUIRED)
  _find_package(rcl_interfaces REQUIRED)
endif()

# Boost
message(STATUS "Find Boost (REQUIRED) ...")
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
_find_package(
  Boost
  REQUIRED
  COMPONENTS
  system
  filesystem
  program_options
  iostreams
  regex)

message(STATUS "Find cnr_yaml (REQUIRED) ...")
_find_package(cnr_yaml REQUIRED)

## In the case the cnr_yamlDependencies is imported from CMakeLists.txt of the 
## cnr_yaml project, the following variables are defined
## Otherwise, they are defined in the cnr_paramConfig.cmake generated in the
## install configuration steps
if(${PROJECT_NAME} STREQUAL "cnr_param")

  if(NOT ROS1_MODULE) 
    if(BUILD_AS_A_CATKIN_PACKAGE)
      _find_package(catkin REQUIRED)
    endif()
  elseif(ROS2_MODULE)
    if(BUILD_AS_AN_AMENT_PACKAGE)
      _find_package(ament_cmake REQUIRED)
    endif()
  endif()

  # Keys: DEPENDENCIES_INCLUDE_DIRS
  # ##############################################################
  # DEPENDENCIES_LINK_LIBRARIES
  # ##############################################################
  list(APPEND DEPENDENCIES_INCLUDE_DIRS "${cnr_yaml_INCLUDE_DIRS}" "${Boost_INCLUDE_DIRS}")

  if(ROS1_MODULE)
    list(APPEND DEPENDENCIES_INCLUDE_DIRS "${catkin_INCLUDE_DIRS}")
  elseif(ROS2_MODULE)
    list(APPEND DEPENDENCIES_INCLUDE_DIRS "${rclcpp_INCLUDE_DIRS}"
        "${rmw_INCLUDE_DIRS}" "${rosidl_runtime_c_INCLUDE_DIRS}"
        "${rcl_interfaces_INCLUDE_DIRS}")
  endif()

  if(cnr_yaml_HAS_NAMESPACE)
    list(APPEND DEPENDENCIES_TARGETS cnr_yaml::cnr_yaml)
  else()
    list(APPEND DEPENDENCIES_TARGETS cnr_yaml)
  endif()

  # Boost
  list(
    APPEND
    DEPENDENCIES_TARGETS
    Boost::system
    Boost::filesystem
    Boost::program_options
    Boost::iostreams
    Boost::regex)

  if(ROS1_MODULE)
    list(APPEND DEPENDENCIES_LINK_LIBRARIES "${catkin_LIBRARIES}")
  endif()

  if(ROS2_MODULE)
    list(APPEND DEPENDENCIES_LINK_LIBRARIES "${rclcpp_LIBRARIES}")
  endif()

endif()