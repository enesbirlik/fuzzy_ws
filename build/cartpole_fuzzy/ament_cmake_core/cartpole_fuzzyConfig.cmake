# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cartpole_fuzzy_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cartpole_fuzzy_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cartpole_fuzzy_FOUND FALSE)
  elseif(NOT cartpole_fuzzy_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cartpole_fuzzy_FOUND FALSE)
  endif()
  return()
endif()
set(_cartpole_fuzzy_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cartpole_fuzzy_FIND_QUIETLY)
  message(STATUS "Found cartpole_fuzzy: 0.4.0 (${cartpole_fuzzy_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cartpole_fuzzy' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cartpole_fuzzy_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cartpole_fuzzy_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cartpole_fuzzy_DIR}/${_extra}")
endforeach()
