#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "thur::thur" for configuration ""
set_property(TARGET thur::thur APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(thur::thur PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libthur.so"
  IMPORTED_SONAME_NOCONFIG "libthur.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS thur::thur )
list(APPEND _IMPORT_CHECK_FILES_FOR_thur::thur "${_IMPORT_PREFIX}/lib/libthur.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
