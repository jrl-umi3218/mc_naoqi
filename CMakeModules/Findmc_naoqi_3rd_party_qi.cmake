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
  # Search using pkgconfig (e.g installed with ros-melodic-naoqi-libqi ros-melodic-naoqi-libqicore)
  include(FindPkgConfig)
  pkg_search_module(LIBQI naoqi_libqi naoqi_libqicore)
  if(LIBQI_FOUND)
    foreach(LIB ${LIBQI_LIBRARIES})
      find_library(${LIB}_FULL_PATH NAME ${LIB} HINTS ${LIBQI_LIBRARY_DIRS})
      list(APPEND LIBQI_FULL_LIBRARIES ${${LIB}_FULL_PATH})
    endforeach()
    message("-- Found libqi using pkgconfig - libraries: ${LIBQI_FULL_LIBRARIES}")
    message("-- Found libqi using pkgconfig - include directories: ${LIBQI_INCLUDE_DIRS}")
    add_library(qi::qi INTERFACE IMPORTED)
    set_target_properties(qi::qi PROPERTIES
      INTERFACE_LINK_LIBRARIES "${LIBQI_FULL_LIBRARIES}"
      INTERFACE_INCLUDE_DIRECTORIES "${LIBQI_INCLUDE_DIRS}"
    )
  else()
    # Look for libqi (as installed from lastest souce)
    find_package(qi QUIET)
    if(${QI_PACKAGE_FOUND})
      add_library(qi::qi INTERFACE IMPORTED)
      message("-- Found libqi using cmake - libraries: ${QI_LIBRARIES}")
      message("-- Found libqi using cmake - include directories: ${QI_INCLUDE_DIRS}")
      set_target_properties(qi::qi PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES ${QI_INCLUDE_DIRS}
        INTERFACE_LINK_LIBRARIES ${QI_LIBRARIES}
      )
    else()
      message(FATAL_ERROR "Could not find the libqi package using CMake package or pkg-config")
    endif()
  endif()
endif()