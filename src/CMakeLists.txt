# Create a executable named mc_rtc_nao
# with the source file: main.cpp
qi_create_bin(mc_rtc_nao main.cpp MCControlNAO.cpp)
qi_use_lib(mc_rtc_nao ALCOMMON ALPROXIES)
# qi_use_lib(mc_rtc_nao ALCOMMON)
pkg_config_use_dependency(mc_rtc_nao mc_rtc)
pkg_config_use_dependency(mc_rtc_nao jsoncpp)

