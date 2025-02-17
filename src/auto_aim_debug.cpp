#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/auto_shoot_aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::CBoard & cboard = io::CBoard::get_instance(config_path);
  io::Camera camera(config_path);

  auto_aim::Detector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path, solver);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    // recorder.record(img, q, t);
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    if (mode == io::Mode::idle) aimer.clear_last();
    /// 自瞄核心逻辑

    solver.set_R_gimbal2world(q);

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);

    auto command = aimer.aim(targets, armors, t, cboard.bullet_speed);

    q = cboard.latest();
    Eigen::Vector3d ypr = tools::eulers(solver.q_to_R_gimbal2world(q), 2, 1, 0);
    double gimbal_yaw = ypr[0];
    double gimbal_pitch = -ypr[1];

    // aimer瞄准位置
    auto opt_yp_should = aimer.yp_should;
    cboard.send(command);

    /// 调试输出

    // tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
    }

    for (auto target : targets) {
      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (int i = 0; i < armor_xyza_list.size(); ++i) {
        Eigen::Vector4d xyza = armor_xyza_list[i];
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        if (i == aimer.aim_id_)
          tools::draw_points(img, image_points, {0, 0, 255});
        else
          tools::draw_points(img, image_points, {0, 255, 0});
        tools::draw_text(img, fmt::format("{}", i), image_points[2]);
      }

      // aimer瞄准位置
      auto aim_point = aimer.yp_should;
      if (aim_point.has_value()) {
        double yaw = opt_yp_should.value().x(), pitch = opt_yp_should.value().y();
        auto image_point = solver.reproject_gimbal(yaw, pitch);
        tools::draw_circle(img, image_point, color::GREEN);  // 绿色为理想瞄准位置
      }
      if (command.control) {
        auto image_point = solver.reproject_gimbal(command.yaw, command.pitch);
        tools::draw_circle(img, image_point, color::RED, 5, 3);  // 红色为发送瞄准位置
      }

      // 云台响应情况
      Eigen::Quaterniond latest = cboard.latest();
      Eigen::Vector3d ypr = tools::eulers(solver.q_to_R_gimbal2world(latest), 2, 1, 0);
      double gimbal_yaw = ypr[0];
      double gimbal_pitch = -ypr[1];
      auto image_point = solver.reproject_gimbal(gimbal_yaw, gimbal_pitch);
      tools::draw_circle(img, image_point, color::YELLOW, 3, 3);  // 黄色为云台实际位置

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;
    }

    // 云台响应情况
    Eigen::Vector3d gimbal_ypr = solver.gimbal_ypr_latest();
    data["gimbal_yaw"] = gimbal_ypr[0] * 57.3;
    data["gimbal_pitch"] = -gimbal_ypr[1] * 57.3;

    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
    }

    auto yp_should = aimer.yp_should;
    if (yp_should.has_value()) {
      double yaw = opt_yp_should.value().x(), pitch = opt_yp_should.value().y();
      data["yaw_should"] = yaw * 57.3;
      data["pitch_should"] = pitch * 57.3;
    }

    data["cmd_shoot"] = command.shoot;
    plotter.plot(data);

    // cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}