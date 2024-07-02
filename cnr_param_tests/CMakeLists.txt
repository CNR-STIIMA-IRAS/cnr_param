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
# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)
 
# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
 
# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# ##############################################################################
# END - PRELIMINARIES     ##
# ##############################################################################

# ##############################################################################
# DEPENDENCIES            ##
# ##############################################################################
find_package(cnr_yaml QUIET)
if(NOT ${cnr_yaml_FOUND})
  message("Check alternative path: ${CMAKE_INSTALL_PREFIX}/..")
  find_package(cnr_yaml REQUIRED PATHS ${CMAKE_INSTALL_PREFIX}/.. NO_DEFAULT_PATH)  
endif()

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



## Catkin is a fake dependency. It is used to test the cmake configuration
if(${catkin_FOUND})
  set(PACKAGE_LIB_DESTINATION     "${CATKIN_PACKAGE_LIB_DESTINATION}")
  set(PACKAGE_BIN_DESTINATION     "${CATKIN_PACKAGE_BIN_DESTINATION}")
  set(PACKAGE_INCLUDE_DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
  set(CONFIG_INSTALL_DIR           "share/${PROJECT_NAME}/cmake_alternative")
else()
  set(PACKAGE_LIB_DESTINATION      "${CMAKE_INSTALL_PREFIX}/lib")
  set(PACKAGE_BIN_DESTINATION      "${CMAKE_INSTALL_PREFIX}/bin")
  set(PACKAGE_INCLUDE_DESTINATION  "${CMAKE_INSTALL_PREFIX}/include")
  set(CONFIG_INSTALL_DIR           "share/${PROJECT_NAME}/cmake")
endif()

find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()
include(CTest REQUIRED)

# ##############################################################################
# END DEPENDANCIES        ##
# ##############################################################################

# ##############################################################################
# GOOGLE TESTS
# ##############################################################################

# EXECUTABLE ###############################################################
get_target_property(cnr_param_location cnr_yaml::cnr_yaml_static IMPORTED_LOCATION_NOCONFIG)
message(STATUS "cnr_param_location: ${cnr_param_location}")
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
               ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mapped_file_module.cpp)

cnr_configure_gtest(test_mapped_file cnr_param::cnr_param cnr_param::cnr_param_server_utilities
                    ${CMAKE_CURRENT_SOURCE_DIR}/include include)

target_compile_definitions(test_mapped_file
                           PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")

# EXECUTABLE #############################################################
add_executable(test_cnr_param
               ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cnr_param.cpp)

cnr_configure_gtest(test_cnr_param cnr_param::cnr_param)

target_compile_definitions(test_cnr_param
                           PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")

list(APPEND TARGETS_LIST 
  test_ros2_yaml_formatter
  test_ros2_module
  test_mapped_file
  test_cnr_param
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

# # EXECUTABLE ###########################################################
add_executable(mapped_file_module
               ${CMAKE_CURRENT_SOURCE_DIR}/example/mapped_file_module.cpp)
target_compile_definitions(mapped_file_module
               PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(mapped_file_module cnr_yaml::cnr_yaml cnr_param::cnr_param)

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

cnr_install(
  ${extracted_version}
  "${TARGETS_LIST}"
  ${HEADERS_DIRS}
  launch
  config
)

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
