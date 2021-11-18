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
  find_package(qi QUIET)
  if(NOT ${QI_PACKAGE_FOUND})
    message(FATAL_ERROR "Could not find the qi library")
  else()
    add_library(qi::qi INTERFACE IMPORTED)
    set_target_properties(qi::qi PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES ${QI_INCLUDE_DIRS}
      INTERFACE_LINK_LIBRARIES ${QI_LIBRARIES}
    )
  endif()
endif()