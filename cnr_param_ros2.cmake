# ##############################################################################
# ROS2 MODULE Build ##
# ##############################################################################

### DIRECTORIES: include and src
list(
  APPEND
  ROS2_DEPENDENCIES_INCLUDE_DIRS
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  ${rosidl_runtime_c_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS})
list(APPEND ROS2_BUILD_INTERFACE_INCLUDE_DIRS 
  ${ROS2_DEPENDENCIES_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
list(APPEND ROS2_INSTALL_INTERFACE_INCLUDE_DIRS
  ${ROS2_DEPENDENCIES_INCLUDE_DIRS} include)

### SOURCES
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/ros2)
list(APPEND cnr_param_ros2_SRC ${SRC_DIR}/param.cpp
     ${SRC_DIR}/yaml_formatter.cpp ${SRC_DIR}/param_retriever.cpp)


### LIBRARY ###################################################################
### cnr_param_ros2
add_library (cnr_param_ros2 SHARED ${cnr_param_ros2_SRC})
target_include_directories(
  cnr_param_ros2
  PUBLIC "$<BUILD_INTERFACE:${ROS2_BUILD_INTERFACE_INCLUDE_DIRS}>"
         "$<INSTALL_INTERFACE:${ROS2_INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(cnr_param_ros2 
  PUBLIC cnr_param::cnr_param_core
  PUBLIC ${DEPENDENCIES_ROS_LIBRARIES})
add_library(cnr_param::cnr_param_ros2 ALIAS cnr_param_ros2)
list(APPEND TARGETS_LIST cnr_param_ros2)
###############################################################################

#### EXECECUTABLES ############################################################
### ros2_yaml_converter
add_executable(
  ros2_yaml_converter
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cnr_param/ros2/ros2_yaml_converter.cpp)
target_include_directories(
  ros2_yaml_converter
  PUBLIC "$<BUILD_INTERFACE:${ROS2_BUILD_INTERFACE_INCLUDE_DIRS}>"
         "$<INSTALL_INTERFACE:${ROS2_INSTALL_INTERFACE_INCLUDE_DIRS}>")
target_link_libraries(ros2_yaml_converter
 PUBLIC $<${YAML_CPP_HAS_NAMESPACE}:yaml-cpp::yaml-cpp>
 PUBLIC $<$<NOT:${YAML_CPP_HAS_NAMESPACE}>:yaml-cpp>
 PUBLIC cnr_param::cnr_param_ros2
 PUBLIC Boost::system
 PUBLIC Boost::filesystem
 PUBLIC Boost::program_options 
 PUBLIC Boost::iostreams
 ${DEPENDENCIES_ROS_LIBRARIES})
set_target_properties(ros2_yaml_converter
                      PROPERTIES LINK_FLAGS "-Wl,-rpath,${CNR_INSTALL_LIB_DIR}")
list(APPEND TARGETS_LIST ros2_yaml_converter)
################################################################################


# ##############################################################################
# Testing         ##
# ##############################################################################
if(ENABLE_TESTING)
  find_package(launch_testing_ament_cmake)
  find_package(ament_cmake_gtest REQUIRED)
  
  #### EXECUTABLE ###############################################################
  ament_add_gtest(test_ros2_yaml_formatter
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros2_yaml_formatter.cpp)
  
  gtest_discover_tests(test_ros2_yaml_formatter)
  cnr_configure_gtest(test_ros2_yaml_formatter 
                      cnr_param::cnr_param_ros2
                      ${ROS2_BUILD_INTERFACE_INCLUDE_DIRS}
                      ${ROS2_INSTALL_INTERFACE_INCLUDE_DIRS})
  
  #### EXECECUTABLE ###########################################################
  add_executable(test_ros2_parameters_node
                 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros2_parameters_node.cpp)

  target_include_directories(test_ros2_parameters_node
                             PUBLIC ${rclcpp_INCLUDE_DIRS})
  target_link_libraries(test_ros2_parameters_node ${rclcpp_LIBRARIES})

  #### EXECECUTABLE #############################################################
  add_executable(
    test_ros2_parameters_remote
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros2_parameters_remote.cpp)

  cnr_configure_gtest(test_ros2_parameters_remote cnr_param::cnr_param_ros2 ${CMAKE_CURRENT_SOURCE_DIR}/include include)

  add_launch_test(test/launch/ros2_test.launch.py TIMEOUT 90)

  list(APPEND TARGETS_LIST test_ros2_yaml_formatter test_ros2_parameters_node
       test_ros2_parameters_remote)
endif()
# ##############################################################################
# END Testing     ##
# ##############################################################################
