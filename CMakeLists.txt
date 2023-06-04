cmake_minimum_required(VERSION 3.22)
project(DA_project2)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(DA_project2
        main.cpp Graph.cpp Graph.h menu.cpp menu.h MutablePriorityQueue.h TSP.cpp TSP.h VertexEdge.cpp VertexEdge.h)
