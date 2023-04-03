cmake_minimum_required(VERSION 3.8)

########################################################################################################################
# Metadata
########################################################################################################################
# Read the package manifest.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" package_xml_str)

# Extract project name.
if(NOT package_xml_str MATCHES "<name>([A-Za-z0-9_]+)</name>")
  message(FATAL_ERROR "Could not parse project name from package manifest (aborting)")
else()
  set(extracted_name ${CMAKE_MATCH_1})
endif()

# Extract project version.
if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
  message(FATAL_ERROR "Could not parse project version from package manifest (aborting)")
else()
  set(extracted_version ${CMAKE_MATCH_1})
endif()

########################################################################################################################
# CMake project
########################################################################################################################
project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

########################################################################################################################
# OPTIONS
########################################################################################################################
option(ENABLE_TESTING "ENABLE TESTING" OFF)
option(ENABLE_COVERAGE_TESTING "ENABLE COVERAGE TESTING" OFF)

########################################################################################################################
# Dependencies
#########################################################################################################################
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# ================================================
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options iostreams regex)

if(ENABLE_TESTING)
  message(STATUS "Enable testing enable_testing")
  enable_testing()
  find_package(GTest REQUIRED)

  if(ENABLE_COVERAGE_TESTING AND NOT WIN32)
    set(CMAKE_CXX_FLAGS "-Wno-deprecated-register ${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS_DEBUG "-Wno-deprecated-register -O0 -g -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS_DEBUG}")
  endif(ENABLE_COVERAGE_TESTING AND NOT WIN32)
endif(ENABLE_TESTING)


# ================================================
add_library(cnr_param_utilities SHARED 
  src/${PROJECT_NAME}/cnr_param.cpp
    src/${PROJECT_NAME}/utils/string.cpp
      src/${PROJECT_NAME}/utils/filesystem.cpp
        src/${PROJECT_NAME}/utils/interprocess.cpp
          src/${PROJECT_NAME}/utils/colors.cpp
            src/${PROJECT_NAME}/utils/yaml.cpp)
target_include_directories(cnr_param_utilities PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
      PUBLIC ${yaml-cpp_INCLUDE_DIRS}
        PUBLIC ${Boost_INCLUDE_DIRS}
)
target_link_libraries(cnr_param_utilities
  yaml-cpp Boost::system Boost::filesystem Boost::iostreams Boost::regex Eigen3::Eigen
)
target_compile_definitions(cnr_param_utilities PRIVATE "CNR_PARAM_BUILDING_LIBRARY")
# ================================================



##################################################################################
add_executable(cnr_param_server 
  src/cnr_param_server/param_server.cpp
    src/cnr_param_server/args_parser.cpp
      src/cnr_param_server/yaml_manager.cpp)
target_include_directories(cnr_param_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
      PUBLIC ${yaml-cpp_INCLUDE_DIRS}
        PUBLIC ${Boost_INCLUDE_DIRS}
)
target_link_libraries(cnr_param_server cnr_param_utilities
  yaml-cpp Boost::program_options Boost::system)

install(DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS cnr_param_utilities cnr_param_server
  EXPORT export_cnr_param
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# export targets
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


##########################
## Testing              ##
##########################
if(ENABLE_TESTING)
  include(GoogleTest)
  add_executable(cnr_param_server_test 
    src/cnr_param_server/args_parser.cpp
    src/cnr_param_server/yaml_manager.cpp
    test/test_server.cpp)
  if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
    target_link_libraries(cnr_param_server_test 
      cnr_param_utilities Threads::Threads GTest::gtest_main
        Boost::program_options Boost::system Boost::filesystem Boost::iostreams Boost::regex)
  else()
    target_link_libraries(cnr_param_server_test cnr_param_utilities GTest::gtest_main)
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
