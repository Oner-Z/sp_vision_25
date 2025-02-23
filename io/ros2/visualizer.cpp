#include "visualizer.hpp"
namespace io
{
Visualizer::Visualizer()
{
  rclcpp::init(0, nullptr);

  viz_node_ = std::make_shared<VizNode>();

  spin_thread_ = std::make_unique<std::thread>([this]() { viz_node_->start(); });
}

Visualizer::~Visualizer()
{
  rclcpp::shutdown();
  spin_thread_->join();
}

void Visualizer::visualize(const mono_loc::Arena & arena)
{
  // 创建 Marker 消息
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";                                // 设置坐标系
  marker.header.stamp = rclcpp::Clock().now();                   // 设置时间戳
  marker.ns = "triangles";                                       // 设置命名空间
  marker.id = 0;                                                 // 设置标识符
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;  // 类型设置为三角形列表
  marker.action = visualization_msgs::msg::Marker::ADD;          // 动作为添加
  marker.pose.orientation.w = 1.0;                               // 设置姿态，旋转为零
  marker.scale.x = 1;                                            // 设置缩放
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.r = 1.0;  // 设置颜色为红色
  marker.color.a = 1.0;  // 设置透明度为不透明

  for (const auto & triangle : arena.triangles_) {
    geometry_msgs::msg::Point p1, p2, p3;

    p1.x = triangle.a_[0];
    p1.y = triangle.a_[1];
    p1.z = triangle.a_[2];

    p2.x = triangle.b_[0];
    p2.y = triangle.b_[1];
    p2.z = triangle.b_[2];

    p3.x = triangle.c_[0];
    p3.y = triangle.c_[1];
    p3.z = triangle.c_[2];

    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
  }
  viz_node_->send_data(marker);
}
}  // namespace io
