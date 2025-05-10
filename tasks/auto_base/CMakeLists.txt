cmake_minimum_required(VERSION 3.16)

add_library(auto_base OBJECT 
    hanging_shooter.cpp
)

target_link_libraries(auto_base io )