# ##############################################################################
# YAML MAPPED FILE MODULE Build ##
# ##############################################################################

### DIRECTORIES: include and src
list(APPEND MAPPED_FILE_BUILD_INTERFACE_INCLUDE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
list(APPEND MAPPED_FILE_INSTALL_INTERFACE_INCLUDE_DIRS
      include)


### SOURCES
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/mapped_file)
list(APPEND cnr_param_mapped_file_SRC ${SRC_DIR}/interprocess.cpp)

### LIBRARY ###################################################################
### cnr_param_mapped_file
add_library(cnr_param_mapped_file SHARED ${cnr_param_mapped_file_SRC})
target_include_directories(
  cnr_param_mapped_file
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> )

target_link_libraries(
  cnr_param_mapped_file
  PUBLIC cnr_param::cnr_param_core
  PUBLIC $<${YAML_CPP_HAS_NAMESPACE}:yaml-cpp::yaml-cpp>
  PUBLIC $<$<NOT:${YAML_CPP_HAS_NAMESPACE}>:PkgConfig::yaml-cpp_pkg_config>
  PUBLIC Boost::system
  PUBLIC Boost::filesystem
  PUBLIC Eigen3::Eigen)

set_target_properties(cnr_param_mapped_file PROPERTIES LINK_FLAGS "-Wl,-rpath,${CNR_INSTALL_LIB_DIR}")
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
         Boost::regex)
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

target_link_libraries(cnr_param_server PUBLIC cnr_param::cnr_param_server_utilities)
list(APPEND TARGETS_LIST cnr_param_server)
### cnr_param_server END #######################################################


# #############################################################################
# Testing  ##
# #############################################################################
if(ENABLE_TESTING)

  ### EXECUTABLE
  add_executable(cnr_param_mapped_file_test
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_yaml_server.cpp)

  target_compile_definitions(cnr_param_mapped_file_test
                             PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")


   target_link_libraries(cnr_param_mapped_file_test
     PUBLIC
     cnr_param::cnr_param_server_utilities
     GTest::gtest_main
   )
   gtest_discover_tests(cnr_param_mapped_file_test)
   add_test(NAME cnr_param_mapped_file_test
            COMMAND cnr_param_mapped_file_test)
endif()
