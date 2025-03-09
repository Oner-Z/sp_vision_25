#ifndef IO__VIZ_NODE_HPP
#define IO__VIZ_NODE_HPP

#include <Eigen/Dense>  // For Eigen::Vector3d
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
namespace io
{
class VizNode : public rclcpp::Node
{
public:
  // 构造函数：初始化订阅者和发布者
  VizNode();

  // 析构函数
  ~VizNode();

  // 启动节点的事件循环
  void start();

  // 发送marker数据到话题
  void send_data(const visualization_msgs::msg::MarkerArray & marker);

private:
  // ROS2 发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};

}  // namespace io

#endif  // Publish2Nav_HPP_
