cmake_minimum_required(VERSION 3.8)
project(swarm)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Wextra")

include_directories("C:/Program Files (x86)/CodeBlocks/MinGW/include")
link_directories("C:/Program Files (x86)/CodeBlocks/MinGW/lib")

set(SOURCE_FILES swarm.cpp)

add_executable(swarm ${SOURCE_FILES})
target_link_libraries(swarm sfml-system-d sfml-window-d sfml-graphics-d)
