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

#
# cnr_install_directories
#
macro(cnr_install_directories CNR_INSTALL_INCLUDE_DIR
      CNR_INSTALL_LIB_DIR CNR_INSTALL_BIN_DIR CNR_INSTALL_SHARE_DIR)
  
  if(${USER_BUILD_TOOL} STREQUAL "CATKIN")
    set(${CNR_INSTALL_INCLUDE_DIR}
        ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
    set(${CNR_INSTALL_LIB_DIR} ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
    set(${CNR_INSTALL_BIN_DIR} ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
    set(${CNR_INSTALL_SHARE_DIR} ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})
  else()
    set(${CNR_INSTALL_INCLUDE_DIR} "${CMAKE_INSTALL_PREFIX}/include")
    set(${CNR_INSTALL_LIB_DIR} "${CMAKE_INSTALL_PREFIX}/lib")
    set(${CNR_INSTALL_BIN_DIR} "${CMAKE_INSTALL_PREFIX}/bin")
    set(${CNR_INSTALL_SHARE_DIR} "${CMAKE_INSTALL_PREFIX}/share")
  endif()
endmacro()

#
# ROS1/CATKIN install
#
macro(cnr_install_catkin_policy TARGETS_LIST)
  install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE)

  install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE)
endmacro()

#
# ROS2/CMAKE install
#
macro(cnr_install_cmake_policy_gt_3_0_0 INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
      BIN_INSTALL_DIR PROJECT_VERSION LIBRARY_TARGETS_LIST EXECUTABLE_TARGETS_LIST)

  set(CONFIG_INSTALL_DIR "share/${PROJECT_NAME}/cmake")
  set(CONFIG_INSTALL_DIR_2 "share/${PROJECT_NAME}/cmake_alternative")
  set(CONFIG_NAMESPACE "${PROJECT_NAME}::")
  set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
  set(VERSION_CONFIG "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
  set(PROJECT_CONFIG_OUTPUT "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
  set(PROJECT_CONFIG_INPUT_TEMPLATE "cmake/cnrConfigTemplate.cmake.in")

  message(STATUS "CONFIG_INSTALL_DIR: ${CONFIG_INSTALL_DIR}")
  message(STATUS "CONFIG_NAMESPACE: ${CONFIG_NAMESPACE}")
  message(STATUS "TARGETS_EXPORT_NAME: ${TARGETS_EXPORT_NAME}")
  message(STATUS "VERSION_CONFIG: ${VERSION_CONFIG}")
  message(STATUS "PROJECT_CONFIG_OUTPUT: ${PROJECT_CONFIG_OUTPUT}")
  message(STATUS "PROJECT_CONFIG_INPUT_TEMPLATE: ${PROJECT_CONFIG_INPUT_TEMPLATE}")

  # Parameter template in the PROJECT_CONFIG_INPUT_TEMPLATE
  set(PACKAGE_INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_DIR}")
  set(EXPORTED_LIBRARY_TARGETS_LIST "${LIBRARY_TARGETS_LIST}")
  set(EXPORTED_EXECUTABLE_TARGETS_LIST "${EXECUTABLE_TARGETS_LIST}")
  set(ROS1_MODULE_COMPILED "${ROS1_MODULE}")
  set(ROS2_MODULE_COMPILED "${ROS2_MODULE}")
  set(MAPPED_FILE_MODULE_COMPILED "${MAPPED_FILE_MODULE}")

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

  install(FILES "${PROJECT_CONFIG_OUTPUT}" "${VERSION_CONFIG}"
          DESTINATION "${CONFIG_INSTALL_DIR_2}")

  # Install cmake targets files
 message(STATUS "TARGETS FILE:" ${PROJECT_NAME}Targets.cmake)
  install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${CONFIG_NAMESPACE}"
    DESTINATION "${CONFIG_INSTALL_DIR}"
    FILE ${PROJECT_NAME}Targets.cmake
  )

  list(APPEND TARGETS_LIST ${EXPORTED_LIBRARY_TARGETS_LIST} ${EXPORTED_EXECUTABLE_TARGETS_LIST})
  install(
    TARGETS ${TARGETS_LIST}
    EXPORT ${TARGETS_EXPORT_NAME}
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
    LIBRARY DESTINATION ${LIB_INSTALL_DIR}
    RUNTIME DESTINATION ${BIN_INSTALL_DIR})

endmacro()

macro(
  cnr_install_cmake_policy
  INCLUDE_INSTALL_DIR
  LIB_INSTALL_DIR
  BIN_INSTALL_DIR
  PROJECT_VERSION
  LIBRARY_TARGETS_LIST
  EXECUTABLE_TARGETS_LIST
  HEADERS_DIRS)

  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 3.0)
    cnr_install_cmake_policy_gt_3_0_0(
      ${INCLUDE_INSTALL_DIR} ${LIB_INSTALL_DIR} ${BIN_INSTALL_DIR}
      ${PROJECT_VERSION} "${LIBRARY_TARGETS_LIST}" "${EXECUTABLE_TARGETS_LIST}")
  else()
    list(APPEND TARGETS_LIST ${EXPORTED_LIBRARY_TARGETS_LIST} ${EXPORTED_EXECUTABLE_TARGETS_LIST})
    message(STATUS "TARGETS_LIST: ${TARGETS_LIST}")
    install(
      TARGETS ${TARGETS_LIST}
      ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
      LIBRARY DESTINATION ${LIB_INSTALL_DIR}
      RUNTIME DESTINATION ${BIN_INSTALL_DIR})

  endif()

  foreach(HEADERS_DIR ${HEADERS_DIRS})
    install(
      DIRECTORY ${HEADERS_DIR}/
      DESTINATION ${INCLUDE_INSTALL_DIR}
      FILES_MATCHING
      PATTERN "*.h"
      PATTERN "*.hpp"
      PATTERN ".svn" EXCLUDE)
  endforeach()

endmacro()
