cmake_minimum_required(VERSION 3.16)

add_library(auto_outpost OBJECT 
    shooter.cpp
    command_executor.cpp
)
target_link_libraries(auto_outpost auto_aim)