# ##############################################################################
# Build           ##
# ##############################################################################
list(
  APPEND
  ROS2_DEPENDENCIES_INCLUDE_DIRS
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  ${rosidl_runtime_c_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS})
list(APPEND ROS2_BUILD_INTERFACE_INCLUDE_DIRS ${ROS2_DEPENDENCIES_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
list(APPEND ROS2_INSTALL_INTERFACE_INCLUDE_DIRS
  ${ROS2_DEPENDENCIES_INCLUDE_DIRS} include)

#
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/ros2)
list(APPEND cnr_param_ros2_SRC ${SRC_DIR}/param.cpp
  ${SRC_DIR}/param_retriever.cpp)

add_library(cnr_param_ros2 SHARED ${cnr_param_ros2_SRC})
target_include_directories(
  cnr_param_ros2
  PUBLIC
  "$<BUILD_INTERFACE:${ROS2_BUILD_INTERFACE_INCLUDE_DIRS}>"
  "$<INSTALL_INTERFACE:${ROS2_INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(cnr_param_ros2 PUBLIC ${DEPENDENCIES_ROS_LIBRARIES})
target_compile_definitions(cnr_param_ros2 PUBLIC -DROS2_AVAILABLE)

list(APPEND TARGETS_LIST cnr_param_ros2)
# ##############################################################################
# End Build       ##
# ##############################################################################

# ##############################################################################
# Testing         ##
# ##############################################################################
if(ENABLE_TESTING)
  add_executable(test_ros2_parameters_node
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros2_parameters_node.cpp)
  target_include_directories(test_ros2_parameters_node
    PUBLIC ${rclcpp_INCLUDE_DIRS})
  target_link_libraries(test_ros2_parameters_node ${rclcpp_LIBRARIES})

  ament_add_gtest(
    test_ros2_parameters
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros2_parameters_remote.cpp
    ${cnr_param_ros2_SRC}
    RUNNER
    ${CMAKE_CURRENT_SOURCE_DIR}/test/launch/ros2_test.launch.py)

  target_include_directories(
    test_ros2_parameters
    PUBLIC "$<BUILD_INTERFACE:${ROS2_BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${ROS2_INSTALL_INTERFACE_INCLUDE_DIRS}>")
  target_link_libraries(test_ros2_parameters ${DEPENDENCIES_ROS_LIBRARIES})
  target_compile_definitions(test_ros2_parameters PUBLIC -DROS2_AVAILABLE)
  set_target_properties(
    test_ros2_parameters PROPERTIES LINK_FLAGS
                                    "-Wl,-rpath,${CNR_INSTALL_LIB_DIR}")

  list(APPEND TARGETS_LIST test_ros2_parameters_node test_ros2_parameters)
endif()
# ##############################################################################
# END Testing     ##
# ##############################################################################
