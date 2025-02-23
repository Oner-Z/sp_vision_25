#ifndef PLANE_HPP
#define PLANE_HPP

#include <eigen3/Eigen/Dense>
#include <optional>

#include "ray.hpp"
namespace mono_loc
{
class Triangle
{
public:
  Eigen::Vector3d a_, b_, c_;  // 三个顶点

  Triangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c) : a_(a), b_(b), c_(c) {}

  Eigen::Vector3d normal() const { return (b_ - a_).cross(c_ - a_).normalized(); }

  // Möller–Trumbore intersection algorithm
  // 这个函数直接返回交点，疑似不必要了
  std::optional<Eigen::Vector3d> intersect_point(const Ray & ray) const
  {
    Eigen::Vector3d ab = b_ - a_;
    Eigen::Vector3d ac = c_ - a_;
    Eigen::Vector3d pvec = ray.direction().cross(ac);
    double det = ab.dot(pvec);

    if (det < 1e-6 && det > -1e-6) {
      return std::nullopt;
    }

    double inv_det = 1.0 / det;
    Eigen::Vector3d tvec = ray.origin() - a_;
    double u = tvec.dot(pvec) * inv_det;
    if (u < 0 || u > 1) {
      return std::nullopt;
    }

    Eigen::Vector3d qvec = tvec.cross(ab);
    double v = ray.direction().dot(qvec) * inv_det;
    if (v < 0 || u + v > 1) {
      return std::nullopt;
    }

    double t = ac.dot(qvec) * inv_det;
    return ray.at(t);
  }

  // Möller–Trumbore intersection algorithm
  std::optional<double> intersect_dist(const Ray & ray) const
  {
    Eigen::Vector3d ab = b_ - a_;
    Eigen::Vector3d ac = c_ - a_;
    Eigen::Vector3d pvec = ray.direction().cross(ac);
    double det = ab.dot(pvec);

    if (det < 1e-6 && det > -1e-6) {
      return std::nullopt;
    }

    double inv_det = 1.0 / det;
    Eigen::Vector3d tvec = ray.origin() - a_;
    double u = tvec.dot(pvec) * inv_det;
    if (u < 0 || u > 1) {
      return std::nullopt;
    }

    Eigen::Vector3d qvec = tvec.cross(ab);
    double v = ray.direction().dot(qvec) * inv_det;
    if (v < 0 || u + v > 1) {
      return std::nullopt;
    }

    double t = ac.dot(qvec) * inv_det;
    return t;
  }
};

}  // namespace mono_loc

#endif  // PLANE_HPP