#
# get_project_name
#
macro(get_project_name filename extracted_name extracted_version)
  # Read the package manifest.
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/${filename}" package_xml_str)

  # Extract project name.
  if(NOT package_xml_str MATCHES "<name>([A-Za-z0-9_]+)</name>")
    message(
      FATAL_ERROR
        "Could not parse project name from package manifest (aborting)")
  else()
    set(extracted_name ${CMAKE_MATCH_1})
  endif()

  # Extract project version.
  if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
    message(
      FATAL_ERROR
        "Could not parse project version from package manifest (aborting)")
  else()
    set(extracted_version ${CMAKE_MATCH_1})
  endif()

  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
    cmake_policy(SET CMP0048 OLD)
  else()
    cmake_policy(SET CMP0048 NEW)
  endif()

  if("$ENV{ROS_VERSION}" STREQUAL "1")
    set(USER_BUILD_TOOL "CATKIN")
  elseif("$ENV{ROS_VERSION}" STREQUAL "2")
    set(USER_BUILD_TOOL "AMENT")
  else()
    set(USER_BUILD_TOOL "CMAKE")
  endif()

endmacro()

#
# cnr_set_flags
#
macro(cnr_set_flags)
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W3" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  endif()

  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()

  if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
  endif()

  set(LOCAL_CXX_STANDARD 20)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

    add_compile_options(-Wall -Wextra -Wpedantic -D_TIME_BITS=64
                        -D_FILE_OFFSET_BITS=64)

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_VERSION GREATER 12)
      set(LOCAL_CXX_STANDARD 20)
    elseif(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_CXX_COMPILER_VERSION GREATER 10)
      set(LOCAL_CXX_STANDARD 20)
    endif()

  endif()

  # Default to C++20
  if(NOT CMAKE_CXX_STANDARD)
    message(STATUS "CMAKE CXX STANDARD: ${LOCAL_CXX_STANDARD}")
    set(CMAKE_CXX_STANDARD ${LOCAL_CXX_STANDARD})
  endif()

  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  if(${CMAKE_VERSION} VERSION_GREATER "3.16.0")
    set(THREADS_PREFER_PTHREAD_FLAG ON)
  endif()

  # # use, i.e. don't skip the full RPATH for the build tree
  set(CMAKE_SKIP_BUILD_RPATH FALSE)

  # when building, don't use the install RPATH already
  # (but later on when installing)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
endmacro()

#
# cnr_target_compile_options
#
macro(cnr_target_compile_options TARGET_NAME)

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")

    target_compile_options(
      ${TARGET_NAME}
      PRIVATE -Wall -Wextra -Wunreachable-code -Wpedantic
      PUBLIC $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")

    target_compile_options(
      ${TARGET_NAME}
      PRIVATE -Wweak-vtables -Wexit-time-destructors -Wglobal-constructors
              -Wmissing-noreturn -Wno-gnu-zero-variadic-macro-arguments
      PUBLIC $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

  elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")

    target_compile_options(${TARGET_NAME} PRIVATE /W3)

  endif()
endmacro()

macro(
  cnr_install
  PROJECT_VERSION
  EXECUTABLE_TARGETS_LIST
  HEADERS_DIRS
  TEST_LAUNCH_DIR
  TEST_CONFIG_DIR)

  set(CONFIG_INSTALL_DIR            "share/${PROJECT_NAME}/cmake")
  set(CONFIG_NAMESPACE              "${PROJECT_NAME}::")
  set(TARGETS_EXPORT_NAME           "${PROJECT_NAME}Targets")
  set(VERSION_CONFIG                "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
  set(PROJECT_CONFIG_OUTPUT         "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
  set(PROJECT_CONFIG_INPUT_TEMPLATE "cmake/cnrConfigTemplate.cmake.in")

  # Parameter template in the PROJECT_CONFIG_INPUT_TEMPLATE
  set(EXPORTED_EXECUTABLE_TARGETS_LIST  "${EXECUTABLE_TARGETS_LIST}")
  set(ROS1_MODULE_COMPILED              "${ROS1_MODULE}")
  set(ROS2_MODULE_COMPILED              "${ROS2_MODULE}")
  set(MAPPED_FILE_MODULE_COMPILED       "${MAPPED_FILE_MODULE}")

  message(STATUS "CONFIG_INSTALL_DIR            = ${CONFIG_INSTALL_DIR}")
  message(STATUS "CONFIG_NAMESPACE              = ${CONFIG_NAMESPACE}")
  message(STATUS "TARGETS_EXPORT_NAME           = ${TARGETS_EXPORT_NAME}")
  message(STATUS "VERSION_CONFIG                = ${VERSION_CONFIG}")
  message(STATUS "PROJECT_CONFIG_OUTPUT         = ${PROJECT_CONFIG_OUTPUT}")
  message(STATUS "PROJECT_CONFIG_INPUT_TEMPLATE = ${PROJECT_CONFIG_INPUT_TEMPLATE}")

  include(CMakePackageConfigHelpers)

  # Create the ${PROJECT_NAME}ConfigVersion.cmake
  write_basic_package_version_file(
    "${VERSION_CONFIG}"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

  # Create the ${PROJECT_NAME}Config.cmake using the template
  # ${PROJECT_NAME}Config.cmake.in
  configure_package_config_file(
    "${PROJECT_CONFIG_INPUT_TEMPLATE}" "${PROJECT_CONFIG_OUTPUT}"
    INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}")

  # Install cmake config files
  install(FILES "${PROJECT_CONFIG_OUTPUT}" "${VERSION_CONFIG}"
          DESTINATION "${CONFIG_INSTALL_DIR}")

  # Install cmake targets files
  message(STATUS "TARGETS FILE:" ${PROJECT_NAME}Targets.cmake)
  install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${CONFIG_NAMESPACE}"
    DESTINATION "${CONFIG_INSTALL_DIR}"
    FILE ${PROJECT_NAME}Targets.cmake
  )

  install(
    TARGETS ${EXPORTED_EXECUTABLE_TARGETS_LIST}
    EXPORT ${TARGETS_EXPORT_NAME}
    ARCHIVE DESTINATION ${PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${PACKAGE_BIN_DESTINATION}
  )

  foreach(HEADERS_DIR ${HEADERS_DIRS})
    install(
      DIRECTORY ${HEADERS_DIR}/
      DESTINATION ${PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING
      PATTERN "*.h"
      PATTERN "*.hpp"
      PATTERN ".svn" EXCLUDE)
  endforeach()

  install(DIRECTORY ${TEST_LAUNCH_DIR} DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY ${TEST_CONFIG_DIR} DESTINATION share/${PROJECT_NAME})
endmacro()


macro(cnr_configure_gtest trg deps)

  gtest_discover_tests(${trg})

  target_link_libraries(
    ${trg}
    ${deps}
    Threads::Threads
    GTest::Main
    Boost::program_options
    Boost::system
    Boost::filesystem
    Boost::iostreams
    Boost::regex)

endmacro()


