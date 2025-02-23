#ifndef LOCATOR_HPP
#define LOCATOR_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "arena.hpp"
#include "tools/logger.hpp"
namespace mono_loc
{
class Locator
{
private:
  Arena arena_;
  Eigen::Vector3d t_camera_to_arena_;
  Eigen::Matrix3d R_camera_to_arena_;
  bool ready_ = false;

  cv::Mat camera_matrix_cv_;
  cv::Mat distort_coeffs_cv_;

  Eigen::Matrix3d camera_matrix_eigen_;

  std::vector<cv::Point3d> PnP_points_in_arena_;

public:
  Locator(std::string config_path) : arena_(config_path)
  {
    auto yaml = YAML::LoadFile(config_path);

    auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
    auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
    camera_matrix_eigen_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(camera_matrix_data.data());
    Eigen::Matrix<double, 1, 5> distort_coeffs_eigen(distort_coeffs_data.data());
    cv::eigen2cv(camera_matrix_eigen_, camera_matrix_cv_);
    cv::eigen2cv(distort_coeffs_eigen, distort_coeffs_cv_);
    std::cout << camera_matrix_cv_ << std::endl;
    std::cout << distort_coeffs_cv_ << std::endl;

    // 读入为double型二维数组
    auto PnP_points_in_arena = yaml["PnP_points_in_arena"].as<std::vector<std::vector<double>>>();
    for (auto & point : PnP_points_in_arena) {
      PnP_points_in_arena_.emplace_back(point[0], point[1], point[2]);
    }
  }

  /**
    * @brief 定位
    * 
    * @param points_img 场地上的点拍摄进相机后得到的 像素平面上的点，未去除畸变
    * @return 返回赛场坐标系下的3维坐标，包含z轴信息
  */
  std::vector<Eigen::Vector3d> locate(std::vector<cv::Point2d> points_img)
  {
    if (!ready_) {
      tools::logger()->error("Locator not ready!");
      return {};
    }

    std::vector<Eigen::Vector3d> result;

    std::vector<cv::Point2d> undistort_points = {};
    cv::undistortPoints(
      points_img, undistort_points, camera_matrix_cv_, distort_coeffs_cv_, cv::noArray(),
      camera_matrix_cv_);

    for (auto & point : undistort_points) {
      Eigen::Vector3d point_homogeneous = {point.x, point.y, 1};
      Eigen::Vector3d point_on_normalized_plane =
        camera_matrix_eigen_.inverse() * point_homogeneous;

      Eigen::Vector3d point_in_arena =
        R_camera_to_arena_ * point_on_normalized_plane + t_camera_to_arena_;
      Ray ray(t_camera_to_arena_, point_in_arena);
      auto intersections = arena_.intersections_with(ray);
      if (!intersections.empty()) {
        Eigen::Vector3d intersection = intersections.front();
        result.push_back(intersection);
      }
    }
    return result;
  }

  /**
     * @brief 使用PnP初始化雷达站自身位置，只需调用一次
     * 
     * @param points_img 与 yaml 配置文件中的点 相对应的 像素平面上的点
     */
  void self_pos_init(std::vector<cv::Point2f> points_img)
  {
    if (points_img.size() != PnP_points_in_arena_.size()) {
      tools::logger()->error("pnp point num didn't match!");
      tools::logger()->error("Failed to init locator!");
      /// TODO: 交互式地指导选点。
      exit(1);
    }
    cv::Mat rvec, tvec, rmat;
    std::cout << camera_matrix_cv_ << std::endl;
    std::cout << distort_coeffs_cv_ << std::endl;
    cv::solvePnP(
      PnP_points_in_arena_, points_img, camera_matrix_cv_, distort_coeffs_cv_, rvec, tvec);
    cv::Rodrigues(rvec, rmat);
    cv::cv2eigen(rmat, R_camera_to_arena_);
    cv::cv2eigen(tvec, t_camera_to_arena_);
    ready_ = true;
  }
};

}  // namespace mono_loc

#endif  // LOCATOR_HPP