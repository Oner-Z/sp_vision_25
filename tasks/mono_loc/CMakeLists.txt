cmake_minimum_required(VERSION 3.16)

find_package(ament_cmake)
find_package(rclcpp)
find_package(std_msgs)
find_package(rosidl_typesupport_cpp)
find_package(visualization_msgs)

# 查找 ROS 的依赖

# 包含 ROS 的头文件
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

# message(STATUS "std_msgs_INCLUDE_DIRS: ${std_msgs_INCLUDE_DIRS}")
# message(STATUS "visualization_msgs_INCLUDE_DIRS: ${visualization_msgs_INCLUDE_DIRS}")

add_library(mono_loc OBJECT 
    arena.cpp
)

ament_target_dependencies(mono_loc rclcpp std_msgs visualization_msgs)