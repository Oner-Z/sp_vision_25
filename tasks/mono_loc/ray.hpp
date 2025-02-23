#ifndef RAY_HPP
#define RAY_HPP

#include <eigen3/Eigen/Dense>

namespace mono_loc
{
class Ray
{
private:
  Eigen::Vector3d origin_;
  Eigen::Vector3d direction_;

public:
  //   // 原点 - 方向 构造函数
  //   Ray(Eigen::Vector3d origin, Eigen::Vector3d direction) : origin_(origin), direction_(direction) {}

  // 原点 - 目标点 构造函数
  Ray(Eigen::Vector3d origin, Eigen::Vector3d target, bool normalize = true) : origin_(origin)
  {
    direction_ = target - origin;
    if (normalize) {
      direction_.normalize();
    }
  }

  Eigen::Vector3d origin() const { return origin_; }
  Eigen::Vector3d direction() const { return direction_; }
  Eigen::Vector3d at(double t) const { return origin_ + t * direction_; }
};
}  // namespace mono_loc
#endif  // RAY_HPP