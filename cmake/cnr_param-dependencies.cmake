############################################
## It does takes inspiration from 
## Ton idea is to have a single file that can be included in the CMakeLists.txt of the package
## and in the >package>Config.cmake
## https://discourse.cmake.org/t/how-to-conditionally-call-either-find-pacakge-or-find-dependency/8175
include(CMakeFindDependencyMacro)

if(${PROJECT_NAME} STREQUAL "cnr_param")
  macro(_find_package)
    find_package(${ARGN})
  endmacro()
else()
  include(CMakeFindDependencyMacro)

  macro(_find_package)
    find_dependency(${ARGN})
  endmacro()
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

# cnr_param
_find_package(cnr_yaml REQUIRED)

# ros2 dependencies
if(cnr_param_COMPILE_ROS2_MODULE)
  _find_package(rclcpp REQUIRED)
  _find_package(rmw REQUIRED)
  _find_package(rosidl_runtime_c REQUIRED)
  _find_package(rcl_interfaces REQUIRED)
elseif(cnr_param_BUILD_AS_A_CATKIN_PACKAGE OR BUILD_AS_A_CATKIN_PACKAGE)
  _find_package(catkin REQUIRED COMPONENTS roscpp)
endif()
