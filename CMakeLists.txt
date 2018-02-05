cmake_minimum_required(VERSION 2.8.3)
project(mc_rtc_naoqi_ros)
set(PROJECT_NAME mc_rtc_naoqi_ros)
set(PROJECT_DESCRIPTION mc_rtc_naoqi_ros)
set(PROJECT_URL https://gite.lirmm.fr/atanguy/mc_rtc_nao)
set(PROJECT_DEBUG_POSTFIX "_d")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/eigen.cmake)
include(cmake/pkg-config.cmake)

setup_project()

# add_definitions(-std=c++11)
# libqi fails to compile with -std=c++11,
# See https://github.com/aldebaran/libqi/issues/1 for more details
add_definitions(-std=gnu++11)
# Do not treat warnings as errors
set(CXX_DISABLE_WERROR 1)

search_for_eigen()

add_required_dependency(naoqi_libqi)
add_required_dependency(naoqi_libqicore)
add_required_dependency(mc_rtc)
add_subdirectory(src)

setup_project_finalize()


