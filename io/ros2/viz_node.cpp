#include "io/ros2/viz_node.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>

#include "tools/logger.hpp"

namespace io
{

VizNode::VizNode() : Node("visualization_publisher")
{
  publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10);

  RCLCPP_INFO(this->get_logger(), "VizNode initialized.");
}

VizNode::~VizNode() { RCLCPP_INFO(this->get_logger(), "VizNode shutting down."); }

void VizNode::send_data(const visualization_msgs::msg::MarkerArray & marker)
{
  // 发布消息
  publisher_->publish(marker);

  RCLCPP_INFO(this->get_logger(), "Sent message");
}

void VizNode::start()
{
  RCLCPP_INFO(this->get_logger(), "Starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

}  // namespace io
