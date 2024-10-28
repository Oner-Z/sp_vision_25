cmake_minimum_required(VERSION 3.16)

add_library(auto_aim OBJECT 
    armor.cpp
    classifier.cpp 
    detector.cpp
    solver.cpp
    auto_shoot_aimer.cpp
    target.cpp
    tracker.cpp
)