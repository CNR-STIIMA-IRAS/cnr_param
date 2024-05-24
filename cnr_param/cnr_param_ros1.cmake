# ##############################################################################
# ROS2 MODULE Build ##
# ##############################################################################

### DIRECTORIES: include and src
list(APPEND ROS_BUILD_INTERFACE_INCLUDE_DIRS 
  "${DEPENDENCIES_INCLUDE_DIRS}"
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
list(APPEND ROS_INSTALL_INTERFACE_INCLUDE_DIRS
  "${DEPENDENCIES_INCLUDE_DIRS}" include)

### SOURCES
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/ros)
list(APPEND cnr_param_ros_SRC ${SRC_DIR}/param.cpp)

### LIBRARY ###################################################################
### cnr_param_ros
add_library (cnr_param_ros SHARED ${cnr_param_ros_SRC})
if(INSTALL_INTERFACE)
message(FATAL_ERROR "ROS_INSTALL_INTERFACE_INCLUDE_DIRS: ${ROS_INSTALL_INTERFACE_INCLUDE_DIRS}")
endif()
target_include_directories(
  cnr_param_ros
  PUBLIC "$<BUILD_INTERFACE:${ROS_BUILD_INTERFACE_INCLUDE_DIRS}>"
         "$<INSTALL_INTERFACE:${ROS_INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(cnr_param_ros 
  PUBLIC cnr_param::cnr_param_core
  PUBLIC "${DEPENDENCIES_ROS_LIBRARIES}" )
add_library(cnr_param::cnr_param_ros ALIAS cnr_param_ros)
list(APPEND LIBRARY_TARGETS_LIST cnr_param_ros)
###############################################################################

