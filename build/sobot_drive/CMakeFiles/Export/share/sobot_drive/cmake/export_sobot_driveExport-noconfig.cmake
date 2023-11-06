#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "sobot_drive::sobot_drive" for configuration ""
set_property(TARGET sobot_drive::sobot_drive APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(sobot_drive::sobot_drive PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsobot_drive.so"
  IMPORTED_SONAME_NOCONFIG "libsobot_drive.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS sobot_drive::sobot_drive )
list(APPEND _IMPORT_CHECK_FILES_FOR_sobot_drive::sobot_drive "${_IMPORT_PREFIX}/lib/libsobot_drive.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
