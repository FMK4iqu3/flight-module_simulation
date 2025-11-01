cmake_minimum_required(VERSION 3.10)
project(TEC_Simulation VERSION 0.1 LANGUAGES CXX)

add_executable(TEC_Simulation sim.cpp)

set_target_properties(TEC_Simulation PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
