cmake_minimum_required(VERSION 3.0.0)
project(pea VERSION 0.1.0)
add_executable(${PROJECT_NAME} main.cpp utils/Menu.cpp utils/Timer.cpp utils/RandomGenerator.cpp utils/Tester.cpp Graph.cpp utils/NeighborhoodSolver.cpp)
set(CMAKE_CXX_STANDARD 20)
