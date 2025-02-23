cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(fmt 8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog 1.9 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/mono_loc)
add_subdirectory(tasks/auto_aim)

##################tests##################
# add_executable(yolo11_test tests/yolo11_test.cpp)
# target_link_libraries(yolo11_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)

add_executable(locator_test tests/locator_test.cpp)
target_link_libraries(locator_test io tools ${OpenCV_LIBS} fmt::fmt spdlog::spdlog yaml-cpp nlohmann_json::nlohmann_json mono_loc)

add_executable(viz_test tests/viz_test.cpp)
target_link_libraries(viz_test io tools ${OpenCV_LIBS} fmt::fmt spdlog::spdlog yaml-cpp nlohmann_json mono_loc)