cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long  -pedantic")

project(6502-emu)
add_library(6502-emu src/debugger.cpp src/instruction.cpp)
target_include_directories(6502-emu PUBLIC inc)
set_property(TARGET 6502-emu PROPERTY CXX_STANDARD 17)

install(TARGETS 6502-emu DESTINATION /usr/lib/6502-emu)
install(FILES inc/*.h DESTINATION /usr/include/6502-emu)
# include_directories(sr)

add_subdirectory(test)
