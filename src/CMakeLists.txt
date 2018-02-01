set ( SOURCES
 main.cpp
 MCControlNAO.cpp
)

set ( HEADERS
 MCControlNAO.h
)

add_executable( mc_rtc_naoqi_ros ${SOURCES} ${HEADERS})
target_link_libraries( mc_rtc_naoqi_ros 
  ${catkin_LIBRARIES}
  ${naoqi_libqi_LIBRARIES}
  ${Boost_LIBRARIES}
  )
pkg_config_use_dependency(mc_rtc_naoqi_ros mc_rtc)

