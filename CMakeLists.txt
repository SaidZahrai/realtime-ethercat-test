cmake_minimum_required(VERSION 3.16)
project(realtime_ethercat LANGUAGES C CXX)

# Fix the source path: your tree says src/performnce.cpp
# If you rename to performance.cpp, update this line accordingly.
add_executable(realtime-ethercat
  src/performnce.cpp
)

# Link to SOEM:
# Prefer pkg-config if soem.pc exists, otherwise fall back to find_path/find_library.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(SOEM QUIET soem)
endif()

if(SOEM_FOUND)
  target_include_directories(realtime-ethercat PRIVATE ${SOEM_INCLUDE_DIRS})
  target_compile_options(realtime-ethercat PRIVATE ${SOEM_CFLAGS_OTHER})
  target_link_directories(realtime-ethercat PRIVATE ${SOEM_LIBRARY_DIRS})
  target_link_libraries(realtime-ethercat PRIVATE ${SOEM_LIBRARIES})
else()
  find_path(SOEM_INCLUDE_DIR NAMES soem/soem.h REQUIRED)
  find_library(SOEM_LIBRARY NAMES soem REQUIRED)
  target_include_directories(realtime-ethercat PRIVATE ${SOEM_INCLUDE_DIR})
  target_link_libraries(realtime-ethercat PRIVATE ${SOEM_LIBRARY})
endif()

# IMPORTANT: install relative to prefix; debian/rules sets -DCMAKE_INSTALL_PREFIX=/opt/platform
install(TARGETS realtime-ethercat RUNTIME DESTINATION bin)
