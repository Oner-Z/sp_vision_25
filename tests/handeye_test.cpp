
#include <fmt/core.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include "tasks/auto_aim/solver.hpp"
#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/img_tools.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/handeye.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> world2pixel(
  const std::vector<cv::Point3f> & worldPoints, const Eigen::Matrix3d & R_gimbal2world,const cv::Mat camera_matrix,const cv::Mat distort_coeffs)
{
  Eigen::Matrix3d R_camera2gimbal = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_camera2gimbal = Eigen::Vector3d::Zero();

  Eigen::Matrix3d R_world2camera = R_camera2gimbal.transpose() * R_gimbal2world.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal.transpose() * t_camera2gimbal;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(worldPoints, rvec, tvec, camera_matrix, distort_coeffs, pixelPoints);
  return pixelPoints;
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");
  auto yaml = YAML::LoadFile(config_path);
  auto height = yaml["height"].as<double>();
  auto grid_num = yaml["grid_num"].as<double>();
  auto grid_size = yaml["grid_size"].as<double>();
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix_eigen(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs_eigen(distort_coeffs_data.data());
  cv::Mat camera_matrix;
  cv::Mat distort_coeffs;
  cv::eigen2cv(camera_matrix_eigen, camera_matrix);
  cv::eigen2cv(distort_coeffs_eigen, distort_coeffs);
  io::CBoard cboard("can0");
  io::Camera camera(config_path);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    solver.set_R_gimbal2world(q);
    std::vector<cv::Point3f> points;
    cv::Mat result = img.clone();

    for (int x = 0; x < grid_num; x++) {
      for (int y = 0; y < grid_num; y++) {
        points.emplace_back(x * grid_size, y * grid_size - grid_num * grid_size / 2, -height);
      }
    }
    std::vector<cv::Point2f> projectedPoints = world2pixel(points, solver.R_gimbal2world(),camera_matrix,distort_coeffs);
    for (const auto & point : projectedPoints) 
    {
      tools::logger()->info("x:{} y:{}",point.x,point.y);
      if (point.x >= 0 && point.x < result.cols && point.y >= 0 && point.y < result.rows) {
        cv::circle(result, point, 3, cv::Scalar(255, 255, 255), -1);
      }
    }
    Eigen::Vector3d euler = solver.R_gimbal2world().eulerAngles(2, 1, 0) * 180.0 / M_PI;
    tools::draw_text(result, fmt::format("yaw   {:.2f}", euler[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(result, fmt::format("pitch {:.2f}", euler[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(result, fmt::format("roll  {:.2f}", euler[2]), {40, 120}, {0, 0, 255});
    if (!display) continue;
    cv::imshow("result", result);
    if (cv::waitKey(1) == 'q') break;
  }
}