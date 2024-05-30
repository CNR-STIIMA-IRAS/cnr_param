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
list(APPEND cnr_param_ros_SRC ${SRC_DIR}/param.cpp ${SRC_DIR}/param_dictionary.cpp)

### LIBRARY ###################################################################
### cnr_param_ros
add_library (cnr_param_ros SHARED ${cnr_param_ros_SRC})
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

