#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP
#include "Eigen/Dense"
#include "tasks/mono_loc/arena.hpp"
#include "viz_node.hpp"
namespace io
{
class Visualizer
{
public:
  Visualizer();

  ~Visualizer();

  void visualize(const mono_loc::Arena & arena);

  // void visualize(const std::vector<Eigen::Vector3d> & positions);

private:
  std::shared_ptr<VizNode> viz_node_;
  std::unique_ptr<std::thread> spin_thread_;
};

}  // namespace io
#endif