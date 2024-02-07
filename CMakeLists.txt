#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

get_project_name("package.xml" extracted_name extracted_version)

project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)

cnr_set_flags()
#############################
## END - PRELIMINARIES     ##
#############################


#############################
## OPTIONS                 ##
#############################
option(USE_ROS1                 "ROS SUPPORT" OFF)
option(ENABLE_TESTING           "ENABLE TESTING" OFF)
option(ENABLE_COVERAGE_TESTING  "ENABLE COVERAGE TESTING" OFF)
option(COMPILE_EXAMPLE          "COMPILE THE EXAMPLE" OFF)

if(USE_ROS1)
  set(ENABLE_TESTING ${CATKIN_ENABLE_TESTING})
endif()
#############################
## END - OPTIONS           ##
#############################

#############################
## DEPENDANCIES            ##
#############################
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options iostreams regex)

if(USE_ROS1)
  find_package(catkin REQUIRED COMPONENTS roscpp)
  catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
    CATKIN_DEPENDS roscpp
  ) 
endif()

cnr_install_directories(USE_ROS1 
  CNR_INSTALL_INCLUDE_DIR CNR_INSTALL_LIB_DIR CNR_INSTALL_BIN_DIR CNR_INSTALL_SHARE_DIR)
cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE USE_ROS1)
#############################
## END DEPENDANCIES        ##
#############################

#################
## Build       ##
#################
list(APPEND DEPENDENCIES_INCLUDE_DIRS ${yaml-cpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
if(USE_ROS1)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
endif()

list(APPEND BUILD_INTERFACE_INCLUDE_DIRS ${DEPENDENCIES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
list(APPEND INSTALL_INTERFACE_INCLUDE_DIRS include ${DEPENDENCIES_INCLUDE_DIRS} include)

# ================================================
add_library(cnr_param_utilities SHARED 
  src/${PROJECT_NAME}/cnr_param.cpp
    src/${PROJECT_NAME}/utils/colors.cpp
      src/${PROJECT_NAME}/utils/filesystem.cpp
        src/${PROJECT_NAME}/utils/interprocess.cpp
          src/${PROJECT_NAME}/utils/string.cpp
            src/${PROJECT_NAME}/utils/yaml.cpp
              include/${PROJECT_NAME}/utils/eigen.h)
target_include_directories(cnr_param_utilities PUBLIC
  "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(cnr_param_utilities 
  PUBLIC yaml-cpp
  PUBLIC Boost::system
  PUBLIC Boost::filesystem
)

add_library(cnr_param_server_utilities SHARED 
              src/cnr_param_server/utils/args_parser.cpp
                src/cnr_param_server/utils/yaml_manager.cpp)
target_include_directories(cnr_param_server_utilities PUBLIC
  "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(cnr_param_server_utilities 
  PUBLIC cnr_param_utilities
  PUBLIC Boost::program_options
  PUBLIC Boost::iostreams
  PUBLIC Boost::regex
)

add_executable(cnr_param_server 
  src/cnr_param_server/param_server.cpp
)
target_include_directories(cnr_param_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(cnr_param_server 
  PUBLIC cnr_param_server_utilities
)
#################
## END - Build ##
#################

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CNR_INSTALL_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".git" EXCLUDE
)

install(TARGETS cnr_param_utilities cnr_param_server cnr_param_server_utilities
        EXPORT export_cnr_param
        ARCHIVE DESTINATION ${CNR_INSTALL_LIB_DIR}
        LIBRARY DESTINATION ${CNR_INSTALL_LIB_DIR}
        RUNTIME DESTINATION ${CNR_INSTALL_BIN_DIR}
)

# if(USE_ROS1)
#   install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   PATTERN ".svn" EXCLUDE)
# endif()
###################
## END - Install ##
###################

##################################
## CMAKE CONFIG (IF NOT CATKIN) ##
##################################
if(NOT USE_ROS1) 
  set(export_targets ${export_targets};cnr_param_utilities)
  set(export_targets ${export_targets};cnr_param_server)
  export(EXPORT export_cnr_param
    FILE "${PROJECT_BINARY_DIR}/export_cnr_param.cmake")

  # Create the cnr_paramConfig.cmake
  set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
  configure_file(cnr_paramConfig.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/cnr_paramConfig.cmake" @ONLY)

  # Create the cnr_paramConfigVersion.cmake
  configure_file(cnr_paramConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/cnr_paramConfigVersion.cmake" @ONLY)
  
  install(FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/cnr_paramConfig.cmake"
    "${PROJECT_BINARY_DIR}/cnr_paramConfigVersion.cmake"
    DESTINATION "share/${PROJECT_NAME}/cmake" COMPONENT dev
  )

  install(EXPORT export_cnr_param
    DESTINATION "share/${PROJECT_NAME}/cmake"
    FILE export_cnr_param.cmake
    COMPONENT dev
  )
endif()


##########################
## Testing              ##
##########################
if(ENABLE_TESTING)
  add_executable(cnr_param_server_test 
    src/cnr_param_server/utils/args_parser.cpp
    src/cnr_param_server/utils/yaml_manager.cpp
    test/test_server.cpp)
  if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
    target_link_libraries(cnr_param_server_test cnr_param_utilities
       Threads::Threads GTest::Main Boost::program_options Boost::system Boost::filesystem Boost::iostreams Boost::regex)
  else()
    target_link_libraries(cnr_param_server_test cnr_param_utilities GTest::Main)
    if(THREADS_HAVE_PTHREAD_ARG)
      target_compile_options(cnr_param_server_test PUBLIC "-pthread")
    endif()
    if(CMAKE_THREAD_LIBS_INIT)
      target_link_libraries(cnr_param_server_test "${CMAKE_THREAD_LIBS_INIT}")
    endif()
  endif()
  target_compile_definitions(cnr_param_server_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")  
  gtest_discover_tests(cnr_param_server_test)
 endif(ENABLE_TESTING)


##########################
## Coverage Testing     ##
##########################
if( ENABLE_TESTING AND ENABLE_COVERAGE_TESTING AND NOT WIN32 )
  if(USE_ROS)
    message(STATUS "The coverage testing is enabled")
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME coverage_report
      DEPENDENCIES cnr_param_server_test
    )
  else()
    ## NOTHING TO DO, SINCE GTEST ALREADY CREATES THE COVERAGE
    ## IF THE COMPILING OPTIONS ARE SET
  endif()
endif()
