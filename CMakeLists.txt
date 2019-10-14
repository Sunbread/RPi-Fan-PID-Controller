cmake_minimum_required(VERSION 3.0)
project(RPi-Fan-PID-Controller LANGUAGES C)

add_definitions(-std=c99)
add_definitions(-D_POSIX_C_SOURCE=199309L)
add_definitions(-Wall -Wextra)
add_definitions(-O3)

aux_source_directory(. SOURCES)
add_executable(fan-control ${SOURCES})
target_link_libraries(fan-control m)
target_link_libraries(fan-control bcm2835)

install(TARGETS fan-control RUNTIME DESTINATION bin)
