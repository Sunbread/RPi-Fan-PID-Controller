cmake_minimum_required(VERSION 3.0)
project(RPi-Fan-PID-Controller LANGUAGES C)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR armv7l)
add_definitions(-std=c99 -D_POSIX_C_SOURCE=199309L -Wall -Wextra -O3)

aux_source_directory(. SOURCES)
add_executable(fan-control ${SOURCES})
target_link_libraries(fan-control m)
target_link_libraries(fan-control bcm2835)

install(TARGETS fan-control RUNTIME DESTINATION bin)
