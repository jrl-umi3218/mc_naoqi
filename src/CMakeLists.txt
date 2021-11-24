set ( SOURCES
 main.cpp
 MCControlNAOqi.cpp
)

set ( HEADERS
 MCControlNAOqi.h
)

add_executable(mc_naoqi ${SOURCES} ${HEADERS})
target_link_libraries(mc_naoqi Boost::program_options)
target_link_libraries(mc_naoqi mc_rtc::mc_control)
target_link_libraries(mc_naoqi mc_pepper::devices)
target_link_libraries(mc_naoqi qi::qi)
