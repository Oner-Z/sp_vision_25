#ifndef ARENA_HPP
#define ARENA_HPP

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

#include "triangle.hpp"
namespace radar
{
class Arena
{
private:
  std::vector<Eigen::Vector3d> points_;
  std::vector<Triangle> triangles_;

public:
  Arena(std::string config_path);
  std::vector<Eigen::Vector3d> get_intersections(const Ray & ray) const;
};

}  // namespace radar

#endif  // ARENA_HPP