macro(cnr_install_directories)
# Catkin and Ament are fake dependencies. They are used to test the building environment and
# configure CMake environment consequently. I do not call catkin_package() or ament_package()
# since they introduce also the creation of other files that are not needed in this case.
  if(${catkin_FOUND})
    set(PACKAGE_LIB_DESTINATION     "${CATKIN_GLOBAL_LIB_DESTINATION}")
    set(PACKAGE_BIN_DESTINATION     "${CATKIN_GLOBAL_LIBEXEC_DESTINATION}/${PROJECT_NAME}")
    set(PACKAGE_INCLUDE_DESTINATION "${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
    set(PACKAGE_CONFIG_DESTINATION  "share/${PROJECT_NAME}/cmake_alternative")
  else()
    set(PACKAGE_LIB_DESTINATION     "lib")
    set(PACKAGE_BIN_DESTINATION     "bin")
    set(PACKAGE_INCLUDE_DESTINATION "include")
    set(PACKAGE_CONFIG_DESTINATION  "share/${PROJECT_NAME}/cmake")
  endif()
  message(STATUS "PACKAGE_BIN_DESTINATION     = ${PACKAGE_BIN_DESTINATION}"  )
  message(STATUS "PACKAGE_LIB_DESTINATION     = ${PACKAGE_LIB_DESTINATION}"  )
  message(STATUS "PACKAGE_INCLUDE_DESTINATION = ${PACKAGE_INCLUDE_DESTINATION}"  )
  message(STATUS "PACKAGE_CONFIG_DESTINATION  = ${PACKAGE_CONFIG_DESTINATION}"           )
endmacro()


#
# cnr_install
#
macro(
  cnr_install
  LIBRARY_TARGETS_LIST
  EXECUTABLE_TARGETS_LIST
  HEADERS_DIRS)

  cnr_install_directories()

  list(APPEND TARGETS_LIST ${LIBRARY_TARGETS_LIST} ${EXECUTABLE_TARGETS_LIST})
  
  # We create the congfig files for the package
  set(CONFIG_NAMESPACE               "${PROJECT_NAME}::")
  set(TARGETS_EXPORT_NAME            "${PROJECT_NAME}Targets")
  set(VERSION_CONFIG                 "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
  set(PROJECT_CONFIG_OUTPUT          "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
  set(PROJECT_CONFIG_INPUT_TEMPLATE  "cmake/cnrConfig.cmake.in")

  # Parameter template in the PROJECT_CONFIG_INPUT_TEMPLATE
  list(APPEND DEPENDENCIES_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/${PACKAGE_INCLUDE_DESTINATION}")
  list(REMOVE_DUPLICATES DEPENDENCIES_INCLUDE_DIRS)

  foreach(_LIBRARY_TARGET ${LIBRARY_TARGETS_LIST})
    get_target_property(target_type ${_LIBRARY_TARGET} TYPE)
    if (target_type STREQUAL "SHARED_LIBRARY")
    get_target_property(target_name ${_LIBRARY_TARGET} OUTPUT_NAME)
      list(APPEND DEPENDENCIES_LINK_LIBRARIES "${CMAKE_INSTALL_PREFIX}/${PACKAGE_LIB_DESTINATION}/${CMAKE_SHARED_LIBRARY_PREFIX}${target_name}${CMAKE_SHARED_LIBRARY_SUFFIX}")
    elseif (target_type STREQUAL "STATIC_LIBRARY")
      get_target_property(target_name ${_LIBRARY_TARGET} OUTPUT_NAME)
      list(APPEND DEPENDENCIES_LINK_LIBRARIES "${CMAKE_INSTALL_PREFIX}/${PACKAGE_LIB_DESTINATION}/${CMAKE_STATIC_LIBRARY_PREFIX}${target_name}${CMAKE_STATIC_LIBRARY_SUFFIX}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES DEPENDENCIES_LINK_LIBRARIES)
  
  foreach( _EXE_TARGET ${EXECUTABLE_TARGETS_LIST})
    get_target_property(target_name ${_EXE_TARGET} OUTPUT_NAME)
    list(APPEND DEPENDENCIES_EXE "${CMAKE_INSTALL_PREFIX}/${PACKAGE_LIB_DESTINATION}/${target_name}")
  endforeach()
  list(REMOVE_DUPLICATES DEPENDENCIES_EXE)
   
  set(EXPORTED_TARGET_INCLUDE_DIRS      "${DEPENDENCIES_INCLUDE_DIRS}")
  set(EXPORTED_LIBRARY_TARGETS_LIST     "${DEPENDENCIES_LINK_LIBRARIES}")
  set(EXPORTED_EXECUTABLE_TARGETS_LIST  "${DEPENDENCIES_EXE}")

  set(EXPORTED_LIBRARY_TARGET_RPATH     "${PACKAGE_LIB_DESTINATION}")
  
  ##
  message(STATUS "CONFIG_NAMESPACE                = ${CONFIG_NAMESPACE}"                )
  message(STATUS "TARGETS_EXPORT_NAME             = ${TARGETS_EXPORT_NAME}"             )
  message(STATUS "VERSION_CONFIG                  = ${VERSION_CONFIG}"                  )
  message(STATUS "PROJECT_CONFIG_OUTPUT           = ${PROJECT_CONFIG_OUTPUT}"           )
  message(STATUS "PROJECT_CONFIG_INPUT_TEMPLATE   = ${PROJECT_CONFIG_INPUT_TEMPLATE}"   )

  message(STATUS "EXPORTED_TARGET_INCLUDE_DIRS    = ${EXPORTED_TARGET_INCLUDE_DIRS}"    )
  message(STATUS "EXPORTED_LIBRARY_TARGETS_LIST   = ${EXPORTED_LIBRARY_TARGETS_LIST}"   )
  message(STATUS "EXPORTED_EXECUTABLE_TARGETS_LIST= ${EXPORTED_EXECUTABLE_TARGETS_LIST}")

  # 1 install files
  foreach(HEADERS_DIR ${HEADERS_DIRS})
    install(
      DIRECTORY ${HEADERS_DIR}/
      DESTINATION "${PACKAGE_INCLUDE_DESTINATION}"
      FILES_MATCHING
      PATTERN "*.h"
      PATTERN "*.hpp"
      PATTERN ".svn" EXCLUDE)
  endforeach()

  install(
    TARGETS ${TARGETS_LIST}
    EXPORT ${TARGETS_EXPORT_NAME}
    ARCHIVE DESTINATION "${PACKAGE_LIB_DESTINATION}"
    LIBRARY DESTINATION "${PACKAGE_LIB_DESTINATION}"
    RUNTIME DESTINATION "${PACKAGE_BIN_DESTINATION}")

  if(ROS1_MODULE)
    catkin_package(
      INCLUDE_DIRS ${INSTALL_INTERFACE_INCLUDE_DIRS}
      LIBRARIES ${LIBRARY_TARGETS_LIST}
      CATKIN_DEPENDS roscpp
      DEPENDS cnr_yaml)
  else()
    include(CMakePackageConfigHelpers)

    #------------------------------------------------------------------------------
    # Configure <PROJECT_NAME>ConfigVersion.cmake common to build and install tree
    write_basic_package_version_file(
      "${VERSION_CONFIG}"
      VERSION ${extracted_version}
      COMPATIBILITY SameMajorVersion)

    #------------------------------------------------------------------------------
    # Create the ${PROJECT_NAME}Config.cmake using the template
    # ${PROJECT_NAME}Config.cmake.in
    configure_package_config_file(
      "${PROJECT_CONFIG_INPUT_TEMPLATE}" "${PROJECT_CONFIG_OUTPUT}"
      INSTALL_DESTINATION "${PACKAGE_CONFIG_DESTINATION}")

    #------------------------------------------------------------------------------
    # Install cmake targets files
    install(
      EXPORT "${TARGETS_EXPORT_NAME}"
      NAMESPACE "${CONFIG_NAMESPACE}"
      DESTINATION "${PACKAGE_CONFIG_DESTINATION}"
      FILE ${TARGETS_EXPORT_NAME}.cmake
    )
    
    #------------------------------------------------------------------------------
    # Install cmake config files
    install(FILES "${PROJECT_CONFIG_OUTPUT}" "${VERSION_CONFIG}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrDependencies.cmake"
            DESTINATION "${PACKAGE_CONFIG_DESTINATION}")
  endif()

endmacro()