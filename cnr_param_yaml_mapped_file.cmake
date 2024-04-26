###################################
## YAML MAPPED FILE MODULE Build ##
###################################
list(APPEND YAML_MAPPED_FILE_DEPENDENCIES_INCLUDE_DIRS ${yaml-cpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
list(APPEND YAML_MAPPED_FILE_BUILD_INTERFACE_INCLUDE_DIRS ${YAML_MAPPED_FILE_DEPENDENCIES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
list(APPEND YAML_MAPPED_FILE_INSTALL_INTERFACE_INCLUDE_DIRS ${YAML_MAPPED_FILE_DEPENDENCIES_INCLUDE_DIRS} include)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/yaml_mapped_file)
list(APPEND cnr_param_yaml_mapped_file_SRC 
        ${SRC_DIR}/interprocess.cpp
          ${SRC_DIR}/yaml.cpp)

add_library(cnr_param_yaml_mapped_file SHARED ${cnr_param_yaml_mapped_file_SRC} )
target_include_directories(cnr_param_yaml_mapped_file PUBLIC
  "$<BUILD_INTERFACE:${YAML_MAPPED_FILE_BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${YAML_MAPPED_FILE_INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(cnr_param_yaml_mapped_file 
  PUBLIC cnr_param::cnr_param_utilities
  PUBLIC $<${YAML_CPP_HAS_NAMESPACE}:yaml-cpp::yaml-cpp>
  PUBLIC $<$<NOT:${YAML_CPP_HAS_NAMESPACE}>:yaml-cpp>
  PUBLIC Boost::system
  PUBLIC Boost::filesystem
  PUBLIC Eigen3::Eigen
)
add_library(cnr_param::cnr_param_yaml_mapped_file ALIAS cnr_param_yaml_mapped_file)

list(APPEND TARGETS_LIST cnr_param_yaml_mapped_file)

add_library(cnr_param_server_utilities SHARED 
              ${SRC_DIR}/args_parser.cpp
                ${SRC_DIR}/yaml_manager.cpp)
target_include_directories(cnr_param_server_utilities PUBLIC
  "$<BUILD_INTERFACE:${YAML_MAPPED_FILE_BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${YAML_MAPPED_FILE_INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(cnr_param_server_utilities 
  PUBLIC cnr_param_yaml_mapped_file
   Boost::program_options
   Boost::iostreams
   Boost::regex
  ${DEPENDENCIES_ROS_LIBRARIES}
)
list(APPEND TARGETS_LIST cnr_param_server_utilities)

add_executable(cnr_param_server 
  ${SRC_DIR}/param_server.cpp
)
target_include_directories(cnr_param_server PUBLIC
  "$<BUILD_INTERFACE:${YAML_MAPPED_FILE_BUILD_INTERFACE_INCLUDE_DIRS}>"
  "$<INSTALL_INTERFACE:${YAML_MAPPED_FILE_INSTALL_INTERFACE_INCLUDE_DIRS}>"
)

target_link_libraries(cnr_param_server 
  PUBLIC cnr_param_server_utilities
)
list(APPEND TARGETS_LIST cnr_param_server)



##########################
## Testing              ##
##########################
if(ENABLE_TESTING)

  add_executable(cnr_param_yaml_mapped_file_test
    ${SRC_DIR}/args_parser.cpp
    ${SRC_DIR}/yaml_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_yaml_server.cpp)
  gtest_discover_tests(cnr_param_yaml_mapped_file_test)

  if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
    target_link_libraries(cnr_param_yaml_mapped_file_test  cnr_param_yaml_mapped_file
      Threads::Threads GTest::Main Boost::program_options Boost::system Boost::filesystem Boost::iostreams Boost::regex)
  else()
    target_link_libraries(cnr_param_yaml_mapped_file_test cnr_param_yaml_mapped_file GTest::Main)
    if(THREADS_HAVE_PTHREAD_ARG)
      target_compile_options(cnr_param_yaml_mapped_file_test PUBLIC "-pthread")
    endif(THREADS_HAVE_PTHREAD_ARG)
    if(CMAKE_THREAD_LIBS_INIT)
      target_link_libraries(cnr_param_yaml_mapped_file_test "${CMAKE_THREAD_LIBS_INIT}")
    endif(CMAKE_THREAD_LIBS_INIT)
  endif()

  target_compile_definitions(cnr_param_yaml_mapped_file_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")  
  target_include_directories(cnr_param_yaml_mapped_file_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif(ENABLE_TESTING)

