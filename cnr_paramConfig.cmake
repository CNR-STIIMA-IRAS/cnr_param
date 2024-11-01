# - Config file for the cnr_param package
# It defines the following variables
#  cnr_param_INCLUDE_DIRS - include directories for FooBar
#  cnr_param_LIBRARIES    - libraries to link against
#  cnr_param_EXECUTABLE   - the bar executable

set(cnr_param_INCLUDE_DIRS "")


include(CMakeFindDependencyMacro)
find_dependency(Boost REQUIRED COMPONENTS system filesystem program_options iostreams regex)
find_dependency(yaml-cpp REQUIRED)
find_dependency(Eigen3 REQUIRED COMPONENTS Core Geometry)

find_dependency(yaml-cpp REQUIRED)
if(yaml-cpp VERSION_LESS "0.8")
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(yaml-cpp_pkg_config REQUIRED yaml-cpp IMPORTED_TARGET)
endif()

# These are IMPORTED targets created by cnr_paramTargets.cmake
set(cnr_param_LIBRARIES cnr_param_core)

set(cnr_param_EXECUTABLE cnr_param_server)

# Our library dependencies (contains definitions for IMPORTED targets)
include("${CMAKE_CURRENT_LIST_DIR}/cnr_paramTargets.cmake")
