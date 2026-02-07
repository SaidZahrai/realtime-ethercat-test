cmake_minimum_required(VERSION 3.16)
project(realtime_ethercat C CXX)
add_executable(realtime_performance crc/performance.cpp)
install(TARGETS realtime-realtime_performance DESTINATION /opt/platform/bin)
