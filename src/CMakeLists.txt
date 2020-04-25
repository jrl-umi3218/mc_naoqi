set ( SOURCES
 main.cpp
 MCControlNAOqi.cpp
 ContactForcePublisher.cpp
)

set ( HEADERS
 MCControlNAOqi.h
 ContactForcePublisher.h
)

add_executable( mc_naoqi ${SOURCES} ${HEADERS})
target_link_libraries(mc_naoqi Boost::program_options)
target_link_libraries(mc_naoqi mc_rtc::mc_control)
target_link_libraries(mc_naoqi mc_rtc::mc_rtc_ros)
pkg_config_use_dependency(mc_naoqi naoqi_libqi)
pkg_config_use_dependency(mc_naoqi naoqi_libqicore)
