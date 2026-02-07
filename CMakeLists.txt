cmake_minimum_required(VERSION 3.16)
project(realtime_ethercat LANGUAGES C CXX)

add_executable(realtime-ethercat
  src/performnce.cpp
)

# Prefer pkg-config (if soem.pc exists), otherwise fall back to manual discovery.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(SOEM QUIET soem)
endif()

if(SOEM_FOUND)
  message(STATUS "SOEM via pkg-config:")
  message(STATUS "  cflags:  ${SOEM_CFLAGS}")
  message(STATUS "  libs:    ${SOEM_LIBRARIES}")
  message(STATUS "  libdirs: ${SOEM_LIBRARY_DIRS}")
  target_include_directories(realtime-ethercat PRIVATE ${SOEM_INCLUDE_DIRS})
  target_link_directories(realtime-ethercat PRIVATE ${SOEM_LIBRARY_DIRS})
  target_link_libraries(realtime-ethercat PRIVATE ${SOEM_LIBRARIES})
  target_compile_options(realtime-ethercat PRIVATE ${SOEM_CFLAGS_OTHER})
else()
  find_path(SOEM_INCLUDE_DIR NAMES soem/soem.h REQUIRED)
  find_library(SOEM_LIBRARY NAMES soem REQUIRED)
  message(STATUS "SOEM via find_*:")
  message(STATUS "  include: ${SOEM_INCLUDE_DIR}")
  message(STATUS "  library: ${SOEM_LIBRARY}")
  target_include_directories(realtime-ethercat PRIVATE ${SOEM_INCLUDE_DIR})
  target_link_libraries(realtime-ethercat PRIVATE ${SOEM_LIBRARY})
endif()

# Install relative to prefix; debian/rules passes -DCMAKE_INSTALL_PREFIX=/opt/platform
install(TARGETS realtime-ethercat RUNTIME DESTINATION bin)
