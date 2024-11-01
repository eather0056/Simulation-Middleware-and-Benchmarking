# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_wander_and_avoid_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED wander_and_avoid_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(wander_and_avoid_FOUND FALSE)
  elseif(NOT wander_and_avoid_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(wander_and_avoid_FOUND FALSE)
  endif()
  return()
endif()
set(_wander_and_avoid_CONFIG_INCLUDED TRUE)

# output package information
if(NOT wander_and_avoid_FIND_QUIETLY)
  message(STATUS "Found wander_and_avoid: 0.0.0 (${wander_and_avoid_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'wander_and_avoid' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${wander_and_avoid_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(wander_and_avoid_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${wander_and_avoid_DIR}/${_extra}")
endforeach()
