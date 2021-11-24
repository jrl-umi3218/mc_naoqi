#
# Copyright 2021 CNRS-UM LIRMM
#
# Try to find the libqi library
#
# In order:
# - try to find_package(qi)
#
# If the library is found, then you can use the qi::qi target
#

if(NOT TARGET qi::qi)
  # Look for libqi (as installed from lastest souce)
  find_package(qi QUIET)
  if(NOT ${QI_PACKAGE_FOUND})
    # If not found, search using pkgconfig (e.g installed with ros-melodic-naoqi-libqi ros-melodic-naoqi-libqicore)
    include(FindPkgConfig)
    pkg_search_module(LIBQI naoqi_libqi naoqi_libqicore)
    if(NOT LIBQI_FOUND)
      message(FATAL_ERROR "Could not find the libqi package using CMake package or pkg-config")
    endif()
    foreach(LIB ${LIBQI_LIBRARIES})
      find_library(${LIB}_FULL_PATH NAME ${LIB} HINTS ${LIBQI_LIBRARY_DIRS})
      list(APPEND LIBQI_FULL_LIBRARIES ${${LIB}_FULL_PATH})
    endforeach()
    message("-- Found libqi libraries: ${LIBQI_FULL_LIBRARIES}")
    add_library(qi::qi INTERFACE IMPORTED)
    set_target_properties(qi::qi PROPERTIES
      INTERFACE_LINK_LIBRARIES "${LIBQI_FULL_LIBRARIES}"
    )
    if(NOT "${LIBQI_INCLUDE_DIRS}" STREQUAL "")
      set_target_properties(qi::qi PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${LIBQI_INCLUDE_DIRS}"
      )
      message("-- Found libqi include directories: ${LIBQI_INCLUDE_DIRS}")
    endif()
  else()
    add_library(qi::qi INTERFACE IMPORTED)
    set_target_properties(qi::qi PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES ${QI_INCLUDE_DIRS}
      INTERFACE_LINK_LIBRARIES ${QI_LIBRARIES}
    )
  endif()
endif()