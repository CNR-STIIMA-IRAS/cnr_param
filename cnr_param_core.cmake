# ##############################################################################
# UTILS Build                   ##
# ##############################################################################
list(APPEND UTILS_BUILD_INTERFACE_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/)
list(APPEND UTILS_INSTALL_INTERFACE_INCLUDE_DIRS
  include)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/core)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/core)
list(APPEND cnr_param_core_SRC ${SRC_DIR}/colors.cpp ${SRC_DIR}/yaml.cpp 
  ${SRC_DIR}/filesystem.cpp ${SRC_DIR}/string.cpp ${INCLUDE_DIR}/eigen.h)

add_library(cnr_param_core SHARED ${cnr_param_core_SRC})

target_include_directories(
  cnr_param_core
  PUBLIC
  "$<BUILD_INTERFACE:${UTILS_BUILD_INTERFACE_INCLUDE_DIRS}>"
  "$<INSTALL_INTERFACE:${UTILS_INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(
  cnr_param_core
  PUBLIC $<${YAML_CPP_HAS_NAMESPACE}:yaml-cpp::yaml-cpp>
  PUBLIC $<$<NOT:${YAML_CPP_HAS_NAMESPACE}>:PkgConfig::yaml-cpp_pkg_config>
  PUBLIC Boost::system
  PUBLIC Boost::filesystem
  PUBLIC Eigen3::Eigen)
add_library(cnr_param::cnr_param_core ALIAS cnr_param_core)

list(APPEND TARGETS_LIST cnr_param_core)
# ##############################################################################
# END - UTILS Build             ##
# ##############################################################################
