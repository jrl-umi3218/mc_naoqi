cmake_minimum_required(VERSION 2.8.3)
project(mc_rtc_naoqi_ros)
set(projectName mc_rtc_naoqi_ros)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  naoqi_libqi
  naoqi_libqicore
)

FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
find_package(EIGEN3 REQUIRED)

#add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})

include_directories(${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set ( SOURCES
 src/main.cpp
 src/MCControlNAO.cpp
)

set ( HEADERS
 src/MCControlNAO.h
)

add_executable( mc_rtc_naoqi_ros ${SOURCES} ${HEADERS})
target_link_libraries( mc_rtc_naoqi_ros 
  ${catkin_LIBRARIES}
  ${naoqi_libqi_LIBRARIES}
  ${Boost_LIBRARIES}
  mc_control
  mc_rbdyn
  mc_rtc_utils)


