#ifndef MONO_LOC__ARENA_HPP
#define MONO_LOC__ARENA_HPP

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

#include "triangle.hpp"
namespace io
{
class Visualizer;
}
namespace mono_loc
{
class Arena
{
private:
  std::vector<Eigen::Vector3d> points_;
  std::vector<Triangle> triangles_;

public:
  Arena(std::string config_path);
  std::vector<Eigen::Vector3d> intersections_with(const Ray & ray) const;

  friend class io::Visualizer;
};

}  // namespace mono_loc

#endif  // ARENA_HPP