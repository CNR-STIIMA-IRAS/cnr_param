
# Get all propreties that cmake supports
if(NOT CMAKE_PROPERTY_LIST)
    execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)
    
    # Convert command output into a CMake list
    string(REGEX REPLACE ";" "\\\\;" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
    string(REGEX REPLACE "\n" ";" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
    list(REMOVE_DUPLICATES CMAKE_PROPERTY_LIST)
endif()
    
function(print_properties)
    message("CMAKE_PROPERTY_LIST = ${CMAKE_PROPERTY_LIST}")
endfunction()
    
function(print_target_properties target)
    if(NOT TARGET ${target})
      message(STATUS "There is no target named '${target}'")
      return()
    endif()

    foreach(property ${CMAKE_PROPERTY_LIST})
        string(REPLACE "<CONFIG>" "${CMAKE_BUILD_TYPE}" property ${property})

        # Fix https://stackoverflow.com/questions/32197663/how-can-i-remove-the-the-location-property-may-not-be-read-from-target-error-i
        if(property STREQUAL "LOCATION" OR property MATCHES "^LOCATION_" OR property MATCHES "_LOCATION$")
            continue()
        endif()

        get_property(was_set TARGET ${target} PROPERTY ${property} SET)
        if(was_set)
            get_target_property(value ${target} ${property})
            message("${target} ${property} = ${value}")
        endif()
    endforeach()
endfunction()

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

