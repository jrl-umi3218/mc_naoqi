cmake_minimum_required(VERSION 3.1.1)

set(CXX_DISABLE_WERROR 1)

project(mc_naoqi)
set(PROJECT_NAME mc_naoqi)
set(PROJECT_DESCRIPTION mc_naoqi)
set(PROJECT_URL https://github.com/jrl-umi3218/mc_naoqi)
set(PROJECT_DEBUG_POSTFIX "_d")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/base.cmake)

setup_project()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)
find_package(Eigen3 REQUIRED)
find_package(mc_rtc 1.4.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options thread)
find_package(mc_pepper REQUIRED)
find_package(mc_naoqi_3rd_party_qi REQUIRED)
add_subdirectory(src)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/etc/"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/mc_naoqi"
        FILES_MATCHING
        PATTERN "*.yaml")

setup_project_finalize()
