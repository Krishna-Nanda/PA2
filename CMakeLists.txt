cmake_minimum_required(VERSION 3.15)
project(PA2)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(PA2
        FeatureGraph.cpp
        FeatureGraph.h
        GraphAnalyzer.cpp
        GraphAnalyzer.h
        GraphHelper.h
        GraphTest.cpp)
