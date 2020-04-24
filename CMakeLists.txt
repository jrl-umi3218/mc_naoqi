cmake_minimum_required(VERSION 3.1.1)
project(mc_rtc_naoqi)
set(PROJECT_NAME mc_rtc_naoqi)
set(PROJECT_DESCRIPTION mc_rtc_naoqi)
set(PROJECT_URL https://gite.lirmm.fr/atanguy/mc_rtc_nao)
set(PROJECT_DEBUG_POSTFIX "_d")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


include(cmake/base.cmake)

setup_project()

# add_definitions(-std=c++11)
# libqi fails to compile with -std=c++11,
# See https://github.com/aldebaran/libqi/issues/1 for more details
add_definitions(-std=gnu++11)
# Do not treat warnings as errors
set(CXX_DISABLE_WERROR 1)

find_package(Eigen3 REQUIRED)
find_package(mc_rtc REQUIRED)
find_package(Boost REQUIRED
	COMPONENTS program_options)
find_package(mc_pepper REQUIRED)

add_required_dependency(naoqi_libqi)
add_required_dependency(naoqi_libqicore)
add_subdirectory(src)

setup_project_finalize()
