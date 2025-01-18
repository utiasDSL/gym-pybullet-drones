# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rrt_path_finder_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rrt_path_finder_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rrt_path_finder_FOUND FALSE)
  elseif(NOT rrt_path_finder_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rrt_path_finder_FOUND FALSE)
  endif()
  return()
endif()
set(_rrt_path_finder_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rrt_path_finder_FIND_QUIETLY)
  message(STATUS "Found rrt_path_finder: 0.0.1 (${rrt_path_finder_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rrt_path_finder' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rrt_path_finder_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rrt_path_finder_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${rrt_path_finder_DIR}/${_extra}")
endforeach()
