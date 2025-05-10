#include "ros2.hpp"
namespace io
{
ROS2::ROS2()
{
  rclcpp::init(0, nullptr);

  subscribe2location_ = std::make_shared<Subscribe2Location>();

  subscribe_spin_thread_ = std::make_unique<std::thread>([this]() { subscribe2location_->start(); });
}

ROS2::~ROS2()
{
  rclcpp::shutdown();
  subscribe_spin_thread_->join();
}


LocationInfo ROS2::subscribe() { return subscribe2location_->subscribe_data(); }

}  // namespace io
