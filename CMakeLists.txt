cmake_minimum_required(VERSION 3.16)
project(realtime_ethercat C CXX)
add_executable(realtime-ethercat src/main.cpp)
install(TARGETS realtime-ethercat DESTINATION /opt/platform/bin)
