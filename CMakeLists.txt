cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long Wno-implicit-fallthrough -pedantic")

project(6502-emu)
add_executable(6502-emu main.cpp instruction.cpp debugger.cpp)
set_property(TARGET 6502-emu PROPERTY CXX_STANDARD 17)
include_directories(.)
