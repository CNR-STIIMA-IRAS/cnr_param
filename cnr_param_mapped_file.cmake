# ##############################################################################
# YAML MAPPED FILE MODULE Build ##
# ##############################################################################

### DIRECTORIES: include and src
list(APPEND MAPPED_FILE_DEPENDENCIES_INCLUDE_DIRS 
      ${yaml-cpp_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
list(APPEND MAPPED_FILE_BUILD_INTERFACE_INCLUDE_DIRS
      ${MAPPED_FILE_DEPENDENCIES_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
list(APPEND MAPPED_FILE_INSTALL_INTERFACE_INCLUDE_DIRS
      ${MAPPED_FILE_DEPENDENCIES_INCLUDE_DIRS} include)

### SOURCES
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/mapped_file)
list(APPEND cnr_param_mapped_file_SRC ${SRC_DIR}/interprocess.cpp)

### LIBRARY ###################################################################
### cnr_param_mapped_file
add_library(cnr_param_mapped_file SHARED ${cnr_param_mapped_file_SRC})
target_include_directories(
  cnr_param_mapped_file
  PUBLIC
    "$<BUILD_INTERFACE:${MAPPED_FILE_BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${MAPPED_FILE_INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(
  cnr_param_mapped_file
  PUBLIC cnr_param::cnr_param_core
  PUBLIC $<${YAML_CPP_HAS_NAMESPACE}:yaml-cpp::yaml-cpp>
  PUBLIC $<$<NOT:${YAML_CPP_HAS_NAMESPACE}>:yaml-cpp>
  PUBLIC Boost::system
  PUBLIC Boost::filesystem
  PUBLIC Eigen3::Eigen)

add_library(cnr_param::cnr_param_mapped_file ALIAS cnr_param_mapped_file)
list(APPEND TARGETS_LIST cnr_param_mapped_file)
### cnr_param_mapped_file END #################################################

### LIBRARY ###################################################################
### cnr_param_server_utilities
add_library(cnr_param_server_utilities SHARED ${SRC_DIR}/args_parser.cpp 
                                              ${SRC_DIR}/yaml_parser.cpp 
                                              ${SRC_DIR}/yaml_manager.cpp)
target_include_directories(
  cnr_param_server_utilities
  PUBLIC
    "$<BUILD_INTERFACE:${MAPPED_FILE_BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${MAPPED_FILE_INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(
  cnr_param_server_utilities
  PUBLIC cnr_param::cnr_param_mapped_file cnr_param::cnr_param_core Boost::program_options Boost::iostreams
         Boost::regex ${DEPENDENCIES_ROS_LIBRARIES})
add_library(cnr_param::cnr_param_server_utilities ALIAS cnr_param_server_utilities)
list(APPEND TARGETS_LIST cnr_param_server_utilities)
### cnr_param_server_utilities END ############################################

### EXECUTABLE ################################################################
### cnr_param_server
add_executable(cnr_param_server ${SRC_DIR}/param_server.cpp)
target_include_directories(
  cnr_param_server
  PUBLIC
    "$<BUILD_INTERFACE:${MAPPED_FILE_BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${MAPPED_FILE_INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(cnr_param_server PUBLIC cnr_param_server_utilities)
list(APPEND TARGETS_LIST cnr_param_server)
### cnr_param_server END #######################################################


# #############################################################################
# Testing  ##
# #############################################################################
if(ENABLE_TESTING)

  ### EXECUTABLE
  add_executable(cnr_param_mapped_file_test 
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_yaml_server.cpp)
  gtest_discover_tests(cnr_param_mapped_file_test)

  cnr_configure_gtest(cnr_param_mapped_file_test cnr_param_server_utilities ${CMAKE_CURRENT_SOURCE_DIR}/include include)
  
  target_compile_definitions(cnr_param_mapped_file_test
                             PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")
endif()
