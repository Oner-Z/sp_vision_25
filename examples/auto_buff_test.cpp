#include <fmt/format.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

// 定义命令行参数
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/standard3.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{@input-path    |                        | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  // 初始化绘图器和退出器
  tools::Plotter plotter;
  tools::Exiter exiter;

  // 构造视频和文本文件路径
  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  // 初始化跟踪器、解算器、追踪器和瞄准器
  auto_buff::Buff_Detector detector(config_path);
  auto_buff::Solver solver(config_path);
  auto_buff::SmallTarget target;
  auto_buff::Aimer aimer(config_path);

  cv::Mat img;
  auto t0 = std::chrono::steady_clock::now();

  // 设置视频起始帧
  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  // 循环处理视频帧
  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    //// 核心逻辑

    solver.set_R_gimbal2world({w, x, y, z});

    // 检测buff
    auto detector_start = std::chrono::steady_clock::now();
    auto power_runes = detector.detect(img);

    // 追踪buff
    auto tracker_start = std::chrono::steady_clock::now();
    solver.solve(power_runes);  // PNP 计算旋转角度

    // 更新目标 瞄准
    auto aimer_start = std::chrono::steady_clock::now();
    target.get_target(power_runes, timestamp);
    auto target_t = target;
    auto command = aimer.aim(target_t, timestamp, 27, false);

    /// 调试输出

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] detector: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, detector_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    nlohmann::json data;

    // buff原始观测数据
    if (power_runes.has_value()) {
      const auto p = power_runes.value();
      data["buff_R_yaw"] = p.ypd_in_world[0] * 57.3;
      data["buff_R_pitch"] = p.ypd_in_world[1] * 57.3;
      data["buff_R_dis"] = p.ypd_in_world[2];

      data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
      data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
      data["buff_roll"] = p.ypr_in_world[2] * 57.3;

      data["buff_R_x"] = p.xyz_in_world[0];
      data["buff_R_y"] = p.xyz_in_world[1];
      data["buff_R_z"] = p.xyz_in_world[2];
      data["buff_R_x2"] = p.blade_xyz_in_world[0];
      data["buff_R_y2"] = p.blade_xyz_in_world[1];
      data["buff_R_z2"] = p.blade_xyz_in_world[2];
    }

    if (!target.is_unsolve()) {
      auto power_rune = power_runes.value();
      // 显示powemechanism
      for (int i = 0; i < 4; i++)
        tools::draw_point(img, power_rune.target.points[i], {0, 255, 0}, 5);

      // 显示pnp解算结果(buff原始观测)
      auto xyz = power_rune.xyz_in_world * 100;  // cm
      auto ypr = power_rune.ypr_in_world * 57.3;
      // clang-format off
      tools::draw_text(img, fmt::format("x{:.0f} y{:.0f} z{:.0f}", xyz[0], xyz[1], xyz[2]),power_rune.r_center, {0, 255, 255}, 0.7, 1);
      tools::draw_text(img, fmt::format("power_rune  yaw{:.1f} pitch{:.1f} roll{:.1f}", ypr[0], ypr[1], ypr[2]),{30, 100}, {0, 255, 255}, 0.7, 1);
      tools::draw_point(img, power_rune.r_center);
  
      // 显示文档欧拉角显示commend欧拉角
      Eigen::Vector3d a = tools::eulers(Eigen::Quaterniond(w, x, y, z), 2, 1, 0);
      tools::draw_text(img,fmt::format("txt  yaw{:.1f} pitch{:.1f} roll{:.1f}", a[0] * 57.3, a[1] * 57.3,a[2] * 57.3),{30, 170}, {0, 255, 255}, 0.7, 1);
      tools::draw_text(img,fmt::format("command  yaw{:.0f} pitch{:.0f}", command.yaw * 57.3, command.pitch * 57.3),{30, 240}, {0, 255, 255}, 0.7, 1);
      // clang-format on

      // 显示预测buff点位
      Eigen::VectorXd x = target.ekf_x();
      double angle = x[6] - aimer.angle;
      cv::Point2f pre_in_img =
        solver.point_buff2pixel(cv::Point3f(0.0, 0.7 * std::sin(angle), 0.7 * std::cos(angle)));
      tools::draw_point(img, pre_in_img, {255, 0, 0}, 5);

      // 观测器内部数据
      data["R_yaw"] = x[0] * 57.3;
      data["R_V_yaw"] = x[1] * 57.3;
      data["R_pitch"] = x[2] * 57.3;
      data["R_dis"] = x[3];
      data["yaw"] = x[4] * 57.3;
      data["pitch"] = x[5] * 57.3;
      data["angle"] = x[6] * 57.3;
      data["spd"] = x[7] * 57.3;
    }

    // 云台响应情况
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
    }

    plotter.plot(data);

    cv::resize(img, img, cv::Size(img.size().width * 0.8, img.size().height * 0.8));
    cv::imshow("result", img);

    int key = cv::waitKey(1);
    if (key == 'q') break;
  }
  cv::destroyAllWindows();
  text.close();  // 关闭文件
  return 0;
}
