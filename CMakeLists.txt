cmake_minimum_required(VERSION 2.8)
project(mc_rtc_nao)
set(PROJECT_NAME mc_rtc_nao)
set(PROJECT_DESCRIPTION mc_rtc_nao)
set(PROJECT_URL ihttps://gite.lirmm.fr/atanguy/mc_rtc_nao)
set(PROJECT_DEBUG_POSTFIX "_d")

include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/eigen.cmake)
include(cmake/pkg-config.cmake)


set(CXX_DISABLE_WERROR 1)

set(DOXYGEN_USE_MATHJAX "YES")
setup_project()

add_definitions(-std=c++11)
find_package(Boost REQUIRED COMPONENTS program_options)

find_package(qibuild)

add_required_dependency("mc_rtc")
# add_required_dependency(tcp-com)
# add_required_dependency(tcp-control)
# add_required_dependency(mc_tcp_msgs)

add_subdirectory(src)

setup_project_finalize()
