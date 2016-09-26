cmake_minimum_required(VERSION 2.8)
project(mc_rtc_nao)

find_package(qibuild)

# Create a executable named mc_rtc_nao
# with the source file: main.cpp
qi_create_bin(mc_rtc_nao "main.cpp")

# Add a simple test:
qi_create_test(test_mc_rtc_nao "test.cpp")
