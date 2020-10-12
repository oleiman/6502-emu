cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR})

project(test)
add_executable(catch src/cpu.cpp)
target_include_directories(catch PRIVATE inc ../inc)
set_property(TARGET catch PROPERTY CXX_STANDARD 17)

set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../extern/catch)
add_library(Catch2::Catch IMPORTED INTERFACE)
set_property(TARGET Catch2::Catch PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${CATCH_INCLUDE_DIR}")

target_link_libraries(catch Catch2::Catch 6502-emu)

# include(CTest)
# include(Catch)
# catch_discover_tests(foo)
