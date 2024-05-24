cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

# ##############################################################################
# PRELIMINARIES           ##
# ##############################################################################
get_project_name("package.xml" extracted_name extracted_version)

project(
  ${extracted_name}
  VERSION ${extracted_version}
  LANGUAGES CXX)

cnr_set_flags()
# ##############################################################################
# END - PRELIMINARIES     ##
# ##############################################################################

# ##############################################################################
# DEPENDENCIES            ##
# ##############################################################################
if("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED)
  find_package(cnr_param REQUIRED)
  include_directories("${cnr_param_INCLUDE_DIRS}")
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  find_package(cnr_param REQUIRED)
else()
  find_package(cnr_param REQUIRED)
endif()



find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()
include(CTest REQUIRED)

cnr_install_directories(CNR_INSTALL_INCLUDE_DIR CNR_INSTALL_LIB_DIR
                        CNR_INSTALL_BIN_DIR CNR_INSTALL_SHARE_DIR)
# ##############################################################################
# END DEPENDANCIES        ##
# ##############################################################################

# ##############################################################################
# GOOGLE TESTS
# ##############################################################################

# EXECUTABLE ###############################################################
add_executable(test_ros2_yaml_formatter
               ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros2_yaml_formatter.cpp)

cnr_configure_gtest(
  test_ros2_yaml_formatter cnr_param::cnr_param)

# EXECUTABLE #############################################################
add_executable(test_ros_module
               ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros_module.cpp)

cnr_configure_gtest(test_ros_module cnr_param::cnr_param
                    ${CMAKE_CURRENT_SOURCE_DIR}/include include)

target_compile_definitions(test_ros_module
                    PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")

# EXECUTABLE #############################################################
add_executable(test_ros2_module
               ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros2_module.cpp)

cnr_configure_gtest(test_ros2_module cnr_param::cnr_param
                    ${CMAKE_CURRENT_SOURCE_DIR}/include include)

target_compile_definitions(test_ros2_module
                    PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")


# EXECUTABLE #############################################################
add_executable(test_mapped_file
               ${CMAKE_CURRENT_SOURCE_DIR}/test/test_,mapped_file_module.cpp)

cnr_configure_gtest(test_mapped_file cnr_param::cnr_param cnr_param::cnr_param_server_utilities
                    ${CMAKE_CURRENT_SOURCE_DIR}/include include)

target_compile_definitions(test_mapped_file
                           PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")

# EXECUTABLE #############################################################
add_executable(test_cnr_param
               ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cnr_param.cpp)

cnr_configure_gtest(test_cnr_param cnr_param::cnr_param
                    ${CMAKE_CURRENT_SOURCE_DIR}/include include)

target_compile_definitions(test_cnr_param
                           PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")

list(APPEND TARGETS_LIST 
  test_ros2_yaml_formatter
  test_ros2_module
  test_mapped_file
  # test_cnr_param
)

# ##############################################################################
# GOOGLE TESTS
# ##############################################################################


# ##############################################################################
# EXAMPLE
# ##############################################################################
# EXECUTABLE ###########################################################
add_executable(ros2_parameters_server_node
               ${CMAKE_CURRENT_SOURCE_DIR}/example/ros2_parameters_server_node.cpp)

target_include_directories(ros2_parameters_server_node
                           PUBLIC ${rclcpp_INCLUDE_DIRS})
target_link_libraries(ros2_parameters_server_node cnr_param::cnr_param ${rclcpp_LIBRARIES})

# EXECUTABLE ###########################################################
add_executable(ros2_parameters_client_node
               ${CMAKE_CURRENT_SOURCE_DIR}/example/ros2_parameters_client_node.cpp)
target_include_directories(ros2_parameters_client_node
                           PUBLIC ${rclcpp_INCLUDE_DIRS})
target_link_libraries(ros2_parameters_client_node cnr_param::cnr_param)

# EXECUTABLE ###########################################################
add_executable(mapped_file_module
               ${CMAKE_CURRENT_SOURCE_DIR}/example/mapped_file_module.cpp)
target_compile_definitions(mapped_file_module
               PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(mapped_file_module cnr_param::cnr_param)

list(APPEND TARGETS_LIST 
  ros2_parameters_server_node
  ros2_parameters_client_node
  mapped_file_module)
# ##############################################################################
# END EXAMPLE
# ##############################################################################

# ##############################################################################
# INSTALL         ##
# ##############################################################################
list(APPEND HEADERS_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(USE_ROS1)
  cnr_install_catkin_policy()
else()
  set(PROJECT_VERSION 1.0.0)
  cnr_install_cmake_policy(
    ${CNR_INSTALL_INCLUDE_DIR}
    ${CNR_INSTALL_LIB_DIR}
    ${CNR_INSTALL_BIN_DIR}
    ${extracted_version}
    "${TARGETS_LIST}"
    ${HEADERS_DIRS}
    launch
    config)
endif()

install(
  PROGRAMS
  launch/ros2_test.launch.py
  DESTINATION lib/${PROJECT_NAME}
)
# ##############################################################################
# END INSTALL     ##
# ##############################################################################


if("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED)
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  ament_package()
endif()
