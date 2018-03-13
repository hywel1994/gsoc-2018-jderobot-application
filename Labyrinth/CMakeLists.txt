cmake_minimum_required(VERSION 3.5)
project(Labyrinth)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")



set(SOURCE_FILES
        main.cpp
        CLabyrinth.cpp
        )

set(HEADER_FILES
        CLabyrinth.h
        )

add_executable(Labyrinth ${SOURCE_FILES} ${HEADER_FILES})
