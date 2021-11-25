set ( SOURCES
 main.cpp
 MCControlNAOqi.cpp
)

set ( HEADERS
 MCControlNAOqi.h
)

add_executable(mc_naoqi ${SOURCES} ${HEADERS})
target_link_libraries(mc_naoqi Boost::program_options)
# XXX this shouldn't be required, looks like boost::thread dependency is missing from libqi target in some cases
target_link_libraries(mc_naoqi Boost::thread)
target_link_libraries(mc_naoqi mc_rtc::mc_control)
target_link_libraries(mc_naoqi mc_pepper::devices)
target_link_libraries(mc_naoqi qi::qi)
