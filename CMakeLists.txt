cmake_minimum_required(VERSION 2.8)
project(mc_rtc_nao)
set(PROJECT_NAME mc_rtc_nao)
set(PROJECT_DESCRIPTION mc_rtc_nao)
set(PROJECT_URL https://gite.lirmm.fr/atanguy/mc_rtc_nao)
set(PROJECT_DEBUG_POSTFIX "_d")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/base.cmake)
# include(cmake/boost.cmake)
include(cmake/eigen.cmake)
include(cmake/pkg-config.cmake)
include(cmake/eigen.cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CXX_DISABLE_WERROR 1)

set(DOXYGEN_USE_MATHJAX "YES")

search_for_eigen()
setup_project()

# add_definitions(-std=c++11)
# NAO SDK fails to compile with -std=c++11,
# See https://github.com/aldebaran/libqi/issues/1 for more details
add_definitions(-std=gnu++11)

# find_package(Boost REQUIRED COMPONENTS program_options)

find_package(qibuild)
search_for_eigen()
add_required_dependency(mc_rtc)
add_required_dependency(mc_rtc_msgs)

add_subdirectory(src)

setup_project_finalize()
