###################################
## UTILS Build                   ##
###################################
list(APPEND UTILS_DEPENDENCIES_INCLUDE_DIRS       ${yaml-cpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
list(APPEND UTILS_BUILD_INTERFACE_INCLUDE_DIRS    ${UTILS_DEPENDENCIES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/)
list(APPEND UTILS_INSTALL_INTERFACE_INCLUDE_DIRS  ${UTILS_DEPENDENCIES_INCLUDE_DIRS} include)

set(SRC_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/utils)
set(INCLUDE_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/utils)
list(APPEND cnr_param_utilities_SRC
    ${SRC_DIR}/colors.cpp
      ${SRC_DIR}/filesystem.cpp
          ${SRC_DIR}/string.cpp
              ${INCLUDE_DIR}/eigen.h)

add_library(cnr_param_utilities SHARED ${cnr_param_utilities_SRC} )
target_include_directories(cnr_param_utilities PUBLIC
  "$<BUILD_INTERFACE:${UTILS_BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${UTILS_INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(cnr_param_utilities 
  PUBLIC Boost::system
  PUBLIC Boost::filesystem
  PUBLIC Eigen3::Eigen
)
add_library(cnr_param::cnr_param_utilities ALIAS cnr_param_utilities)

list(APPEND TARGETS_LIST cnr_param_utilities)
###################################
## END - UTILS Build             ##
###################################

