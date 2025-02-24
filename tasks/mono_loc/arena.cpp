#include "arena.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

#include "tools/logger.hpp"
namespace mono_loc
{

Arena::Arena(std::string config_path)
{
  auto config = YAML::LoadFile(config_path);
  auto path_to_points = config["points"].as<std::string>();
  auto path_to_triangles = config["triangles"].as<std::string>();

  // Load points
  std::ifstream file(path_to_points);
  if (!file.is_open()) {
    tools::logger()->info("[Arena] Failed to open points file.");
  }

  int num_points = 0;
  std::string line;
  while (std::getline(file, line)) {
    if (line[0] == '#') {
      continue;
    }

    size_t pos = 0;
    while ((pos = line.find(',')) != std::string::npos) {
      line.replace(pos, 1, " ");  // 将逗号替换为空格
    }
    std::stringstream ss(line);  // 使用stringstream解析每行数据

    double x, y, z;
    ss >> x >> y >> z;
    points_.emplace_back(x / 1000, y / 1000, z / 1000);  // mm -> m 将点存储到points向量中

    ++num_points;
  }

  tools::logger()->info("[Arena] Constructed with {} points.", num_points);
  // for (auto point : points_) {
  //   std::cout << point << std::endl;
  // }

  // Load triangles
  file = std::ifstream(path_to_triangles);
  if (!file.is_open()) {
    tools::logger()->info("[Arena] Failed to open triangles file.");
  }
  while (std::getline(file, line)) {
    if (line[0] == '#') {
      continue;
    }
    size_t pos = 0;
    while ((pos = line.find(',')) != std::string::npos) {
      line.replace(pos, 1, " ");  // 将逗号替换为空格
    }
    std::stringstream ss(line);
    int a, b, c;
    ss >> a >> b >> c;
    if (a >= points_.size() || b >= points_.size() || c >= points_.size()) {
      tools::logger()->warn("[Arena] Invalid triangle: {}, {}, {}", a, b, c);
      continue;
    }
    Eigen::Vector3d ab = points_[b] - points_[a];
    Eigen::Vector3d ac = points_[c] - points_[a];
    Eigen::Vector3d normal = ab.cross(ac);
    if (normal.dot(Eigen::Vector3d(0, 0, 1)) > 0)
      triangles_.emplace_back(points_[a], points_[b], points_[c]);
    else
      triangles_.emplace_back(points_[b], points_[a], points_[c]);
  }
}

std::vector<Eigen::Vector3d> Arena::intersections_with(const Ray & ray) const
{
  std::vector<double> distances;
  for (const auto & triangle : triangles_) {
    auto dist = triangle.intersect_dist(ray);
    if (dist.has_value()) {
      distances.push_back(dist.value());
    }
  }
  sort(distances.begin(), distances.end());

  std::vector<Eigen::Vector3d> intersections;
  for (auto dist : distances) {
    // tools::logger()->info("dist: {}", dist);
    intersections.push_back(ray.at(dist));
  }
  return intersections;
}
}  // namespace mono_loc
