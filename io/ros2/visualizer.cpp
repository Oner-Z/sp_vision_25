#include "visualizer.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
namespace io
{
Visualizer::Visualizer(bool debug) : debug_(debug)
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
  // 创建 MarkerArray 消息
  visualization_msgs::msg::MarkerArray marker_array;

  // 可视化三角形列表
  visualization_msgs::msg::Marker triangle_marker;
  triangle_marker.header.frame_id = "map";                                // 设置坐标系
  triangle_marker.header.stamp = rclcpp::Clock().now();                   // 设置时间戳
  triangle_marker.ns = "triangles";                                       // 设置命名空间
  triangle_marker.id = 0;                                                 // 设置标识符
  triangle_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;  // 类型设置为三角形列表
  triangle_marker.action = visualization_msgs::msg::Marker::ADD;          // 动作为添加
  triangle_marker.pose.orientation.w = 1.0;                               // 设置姿态，旋转为零
  triangle_marker.scale.x = 1;                                            // 设置缩放
  triangle_marker.scale.y = 1;
  triangle_marker.scale.z = 1;
  triangle_marker.color.r = 0.5;  // 设置颜色为红色
  triangle_marker.color.g = 0.5;  // 设置绿色为 0
  triangle_marker.color.b = 0.5;  // 设置蓝色为 0
  triangle_marker.color.a = 0.7;  // 设置透明度为不透明

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

    triangle_marker.points.push_back(p1);
    triangle_marker.points.push_back(p2);
    triangle_marker.points.push_back(p3);
  }

  // 将三角形标记添加到 MarkerArray 中
  marker_array.markers.push_back(triangle_marker);

  if (debug_) {
    // 可视化顶点编号
    visualization_msgs::msg::Marker point_marker;
    point_marker.header.frame_id = "map";                                   // 设置坐标系
    point_marker.header.stamp = rclcpp::Clock().now();                      // 设置时间戳
    point_marker.ns = "points";                                             // 设置命名空间
    point_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;  // 设置文本标记类型
    point_marker.action = visualization_msgs::msg::Marker::ADD;             // 动作为添加
    point_marker.pose.orientation.w = 1.0;                                  // 设置姿态，旋转为零
    point_marker.scale.z = 0.4;
    point_marker.color.r = 0.0;
    point_marker.color.g = 1.0;
    point_marker.color.b = 0.0;
    point_marker.color.a = 1.0;  // 设置透明度为不透明

    for (int i = 0; i < arena.points_.size(); ++i) {
      point_marker.id = i;  // 每个点一个唯一的 id
      std::cout << "point " << i << ": " << arena.points_[i].transpose() << std::endl;

      geometry_msgs::msg::Point p;
      p.x = arena.points_[i][0];
      p.y = arena.points_[i][1];
      p.z = arena.points_[i][2];
      point_marker.points.push_back(p);

      point_marker.text = std::to_string(i);  // 显示顶点编号
      point_marker.pose.position = p;

      // 将文本标记添加到 MarkerArray 中
      marker_array.markers.push_back(point_marker);
    }

    // 发布顶点编号的 MarkerArray
    viz_node_->send_data(marker_array);
  }
}
}  // namespace io
