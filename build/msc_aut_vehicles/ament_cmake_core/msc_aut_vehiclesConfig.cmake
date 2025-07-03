# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_msc_aut_vehicles_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED msc_aut_vehicles_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(msc_aut_vehicles_FOUND FALSE)
  elseif(NOT msc_aut_vehicles_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(msc_aut_vehicles_FOUND FALSE)
  endif()
  return()
endif()
set(_msc_aut_vehicles_CONFIG_INCLUDED TRUE)

# output package information
if(NOT msc_aut_vehicles_FIND_QUIETLY)
  message(STATUS "Found msc_aut_vehicles: 0.0.0 (${msc_aut_vehicles_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'msc_aut_vehicles' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${msc_aut_vehicles_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(msc_aut_vehicles_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${msc_aut_vehicles_DIR}/${_extra}")
endforeach()
