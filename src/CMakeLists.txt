set ( SOURCES
 main.cpp
 MCControlNAOqi.cpp
)

set ( HEADERS
 MCControlNAOqi.h
)

add_executable( mc_naoqi ${SOURCES} ${HEADERS})
target_link_libraries(mc_naoqi Boost::program_options)
target_link_libraries(mc_naoqi mc_rtc::mc_control)
target_link_libraries(mc_naoqi mc_rtc::mc_rtc_ros)
target_link_libraries(mc_naoqi mc_pepper::devices)
pkg_config_use_dependency(mc_naoqi naoqi_libqi)
pkg_config_use_dependency(mc_naoqi naoqi_libqicore)
