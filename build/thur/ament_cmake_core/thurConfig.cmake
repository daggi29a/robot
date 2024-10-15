# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_thur_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED thur_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(thur_FOUND FALSE)
  elseif(NOT thur_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(thur_FOUND FALSE)
  endif()
  return()
endif()
set(_thur_CONFIG_INCLUDED TRUE)

# output package information
if(NOT thur_FIND_QUIETLY)
  message(STATUS "Found thur: 0.0.0 (${thur_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'thur' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT thur_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(thur_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${thur_DIR}/${_extra}")
endforeach()
