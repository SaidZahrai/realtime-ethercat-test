cmake_minimum_required(VERSION 3.16)
project(realtime_ethercat LANGUAGES C CXX)

add_executable(realtime-ethercat
  src/performnce.cpp
)

# Install relative to prefix; debian/rules passes -DCMAKE_INSTALL_PREFIX=/opt/platform
install(TARGETS realtime-ethercat RUNTIME DESTINATION bin)
