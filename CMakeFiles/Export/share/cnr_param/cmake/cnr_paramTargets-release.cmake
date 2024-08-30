#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cnr_param::cnr_param_core" for configuration "Release"
set_property(TARGET cnr_param::cnr_param_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cnr_param::cnr_param_core PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/lib/libcnr_param_core.so"
  IMPORTED_SONAME_RELEASE "libcnr_param_core.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS cnr_param::cnr_param_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_cnr_param::cnr_param_core "/usr/local/lib/libcnr_param_core.so" )

# Import target "cnr_param::cnr_param" for configuration "Release"
set_property(TARGET cnr_param::cnr_param APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cnr_param::cnr_param PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/lib/libcnr_param.so"
  IMPORTED_SONAME_RELEASE "libcnr_param.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS cnr_param::cnr_param )
list(APPEND _IMPORT_CHECK_FILES_FOR_cnr_param::cnr_param "/usr/local/lib/libcnr_param.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
