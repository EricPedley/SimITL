cmake_minimum_required(VERSION 3.3)
# Headless runner / test harness for the tailsitter wing physics

set(SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

add_executable(simitl-wing
  ${SOURCE_FILES}
)

target_include_directories(simitl-wing PUBLIC
  "../../src"
  "../../extern"
  "../../lib"
  "../simitl-playback"
)

target_compile_features(simitl-wing PUBLIC cxx_std_17)

if (UNIX)
  target_link_libraries(simitl-wing PUBLIC -lpthread -lrt)
endif (UNIX)

if (WIN32)
  target_link_libraries(simitl-wing PUBLIC user32 kernel32)
endif (WIN32)

target_link_libraries(simitl-wing PUBLIC
  fmt-header-only
  SimITL
  sitl
)
