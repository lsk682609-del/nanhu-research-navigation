#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "fmt::fmt" for configuration "None"
set_property(TARGET fmt::fmt APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(fmt::fmt PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/aarch64-linux-gnu/libfmt.so.8.1.1"
  IMPORTED_SONAME_NONE "libfmt.so.8"
  )

list(APPEND _IMPORT_CHECK_TARGETS fmt::fmt )
list(APPEND _IMPORT_CHECK_FILES_FOR_fmt::fmt "${_IMPORT_PREFIX}/lib/aarch64-linux-gnu/libfmt.so.8.1.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
