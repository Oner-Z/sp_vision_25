cmake_minimum_required(VERSION 3.16.3)

project(sp_vision_hero_pneumatic)

set(CMAKE_CXX_STANDARD 17)


find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)

add_executable(test examples/find_lightspot_example.cpp  tasks/hanging_shooter.cpp )
target_link_libraries(test tools io  ${OpenCV_LIBS} fmt::fmt)

add_executable(find_lightspot_test tests/find_lightspot_test.cpp  tasks/hanging_shooter.cpp  tasks/h_solver.cpp)
target_link_libraries(find_lightspot_test tools io  ${OpenCV_LIBS} fmt::fmt)

add_executable(camera_test examples/camera_test.cpp)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)

add_executable(hanging_shooting_example tests/hanging_shooting_example.cpp  tasks/hanging_shooter.cpp  tasks/h_solver.cpp)
target_link_libraries(hanging_shooting_example tools io  ${OpenCV_LIBS} fmt::fmt)

add_executable(shooting_angle_test tests/shooting_angle_test.cpp )
target_link_libraries(shooting_angle_test tools io  ${OpenCV_LIBS} fmt::fmt)


add_executable(hanging_shooting_with_lidar tests/hanging_shooting_with_lidar.cpp  tasks/hanging_shooter.cpp  tasks/h_solver.cpp)
target_link_libraries(hanging_shooting_with_lidar tools io  ${OpenCV_LIBS} fmt::fmt)

add_executable(communicate_test tests/communicate_test.cpp  tasks/hanging_shooter.cpp  tasks/h_solver.cpp)
target_link_libraries(communicate_test tools io  ${OpenCV_LIBS} fmt::fmt)