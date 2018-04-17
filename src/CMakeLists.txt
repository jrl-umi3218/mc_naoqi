set ( SOURCES
 main.cpp
 MCControlNAOqi.cpp
)

set ( HEADERS
 MCControlNAOqi.h
)

add_executable( mc_rtc_naoqi ${SOURCES} ${HEADERS})
target_link_libraries( mc_rtc_naoqi
  ${Boost_LIBRARIES}
  )
pkg_config_use_dependency(mc_rtc_naoqi mc_rtc)
pkg_config_use_dependency(mc_rtc_naoqi naoqi_libqi)
pkg_config_use_dependency(mc_rtc_naoqi naoqi_libqicore)

