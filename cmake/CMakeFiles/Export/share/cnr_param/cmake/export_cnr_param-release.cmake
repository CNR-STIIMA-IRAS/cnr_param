#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cnr_param_utilities" for configuration "Release"
set_property(TARGET cnr_param_utilities APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cnr_param_utilities PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/lib/libcnr_param_utilities.so"
  IMPORTED_SONAME_RELEASE "libcnr_param_utilities.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS cnr_param_utilities )
list(APPEND _IMPORT_CHECK_FILES_FOR_cnr_param_utilities "/usr/local/lib/libcnr_param_utilities.so" )

# Import target "cnr_param_server" for configuration "Release"
set_property(TARGET cnr_param_server APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cnr_param_server PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/bin/cnr_param_server"
  )

list(APPEND _IMPORT_CHECK_TARGETS cnr_param_server )
list(APPEND _IMPORT_CHECK_FILES_FOR_cnr_param_server "/usr/local/bin/cnr_param_server" )

# Import target "cnr_param_server_utilities" for configuration "Release"
set_property(TARGET cnr_param_server_utilities APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cnr_param_server_utilities PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/lib/libcnr_param_server_utilities.so"
  IMPORTED_SONAME_RELEASE "libcnr_param_server_utilities.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS cnr_param_server_utilities )
list(APPEND _IMPORT_CHECK_FILES_FOR_cnr_param_server_utilities "/usr/local/lib/libcnr_param_server_utilities.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
