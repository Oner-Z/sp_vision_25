#include <fmt/core.h>
#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim/classifier.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_outpost/command_executor.hpp"
#include "tasks/auto_aim/yolov8.hpp"
#include "tasks/auto_base/hanging_shooter.hpp"
#include "tasks/auto_outpost/shooter.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{can c          | can0 | can端口名称     }"
  "{exposure-ms e  |  2.0 | 曝光时间，单位ms  }"
  "{gamma g        |  0.5 | 伽马值，基准为1   }"
  "{debug d        |      | 输出调试画面和信息 }"
  "{@config-path   | configs/hero.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // bind_to_p_cores();
  // elevate_priority();

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto debug = cli.has("debug");
  auto can = cli.get<std::string>("can");
  auto exposure_ms = cli.get<double>("exposure-ms");
  auto gamma = cli.get<double>("gamma");
  auto config_path = cli.get<std::string>(0);

  tools::logger()->info("Initialization Start");
  io::CBoard cboard(can);
  tools::logger()->info("Cboard Start Done");
  io::Camera camera(config_path);
  tools::logger()->info("Camera Start Done");
  io::ROS2 ros2;
  tools::logger()->info("ROS2 Start Done");
  // auto_aim::Classifier classifier(config_path);
  // auto_aim::Detector detector(config_path, debug);
  auto_aim::YOLOV8 detector(config_path, debug);
  // auto_aim::multithread::MultiThreadDetector detector(config_path, debug);
  tools::logger()->info("Detector Start Done");
  auto_aim::Solver solver(config_path);
  tools::logger()->info("Solver Start Done");
  auto_aim::Tracker tracker(config_path, solver);
  tools::logger()->info("Tracker Start Done");
  auto_outpost::Shooter shooter(config_path);
  tools::logger()->info("Shooter Start Done");
  auto_base::HangingShooter hangingshooter(config_path);
  tools::logger()->info("HangingShooter Start Done");

  // 独立读图线程和独立识别线程
  // auto detect_thread = std::thread([&]() {
  //   cv::Mat img;
  //   std::chrono::steady_clock::time_point t;

  //   while (!exiter.exit()) {
  //     camera.read(img, t);
  //     detector.push(img, t);
  //   }
  // });
  // tools::logger()->info("Seperate GetImage and Detector Thread Done");

  // 独立决策线程
  auto_outpost::CommandExecutor executor(shooter, cboard);
  executor.start();
  tools::logger()->info("Seperate Command Thread Done");

  Eigen::Quaterniond q;

  // std::list<auto_aim::Armor> armors;
  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  auto last_fps_time = std::chrono::steady_clock::now();
  int frame_counter = 0;
  double fps = 0.0;

  nlohmann::json data;
  
  tools::logger()->info("AUTO AIM START");
  for (int frame_count = 0; !exiter.exit(); frame_count++) {
    // if (debug) {
    //   auto [img, armors, t] = detector.debug_pop();
    // } else {
    //   auto [armors, t] = detector.pop();
    // }
    // auto [img, armors, t] = detector.debug_pop();  // 这是个构造
    camera.read(img, t);
   
    q = cboard.imu_at(t - 1ms);
    if (!img.empty()) recorder.record(img, q, t);

    mode = cboard.mode;  // TODO 
    if (last_mode != mode) tools::logger()->info("Switch to {}", io::MODES[mode]);
    last_mode = mode;
    if(io::MODES[mode] == "hanging_shooting")
    {
        io::Command command = hangingshooter.aim(q,ros2.subscribe(),cboard.bullet_speed);
        cboard.send(command);
    }
    else
    {
        solver.set_R_gimbal2world(q);
        // tools::logger()->debug("114514");
        auto armors = detector.detect(img);
        // tools::logger()->debug("1919810");
        Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
        auto targets = tracker.track(armors, t, ypr[0], true, mode);

        // auto command = shooter.shoot(targets, t, cboard.bullet_speed, true, ypr);
        // cboard.send(command);

        executor.push(targets, t, cboard.bullet_speed, ypr);
        data["fps"] = fps;
        plotter.plot(data);

        if (!debug) {
        frame_counter++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_fps_time).count();
        if (elapsed >= 1.0) {
            fps = frame_counter / elapsed;
            frame_counter = 0;
            last_fps_time = now;
        }
        continue;
        }

        tools::draw_text(img, fmt::format("FPS: {:.1f}", fps), {10, 60}, {255, 255, 0});
        tools::draw_text(img, fmt::format("[{}] [{}]", frame_count, tracker.state_str()), {10, 30}, {255, 255, 255});
        int armor_id = 0;
        if (tracker.state()) {  // tracker.state() && targets.size()
        // 当前帧target更新后
        auto target = tracker.get_target();
        std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
        for (const Eigen::Vector4d & xyza : armor_xyza_list) {
            auto image_points = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
            tools::draw_points(img, image_points, {0, 255, 0});
            tools::draw_text(img, fmt::format("No: {}", armor_id), image_points[0], {0, 255, 255});
            armor_id++;
        }

        // aimer瞄准位置
        auto aim_point = shooter.debug_aim_point_;
        Eigen::Vector4d aim_xyza = aim_point.xyza;
        auto image_points = solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
        if (aim_point.valid)
            tools::draw_points(img, image_points, {0, 0, 255});
        else
            tools::draw_points(img, image_points, {255, 0, 0});

        // 观测器内部数据
        Eigen::VectorXd x = target.ekf_x();
        // x vx y vy z vz a w r l h
        // a: angle
        // w: angular velocity
        data["x"] = x[0];
        data["vx"] = x[1];
        data["y"] = x[2];
        data["vy"] = x[3];
        data["z"] = x[4];
        data["vz"] = x[5];
        data["a"] = x[6] * 57.3;
        data["v"] = sqrt(x[1] * x[1] + x[3] * x[3]);
        data["w"] = x[7];
        data["r"] = x[8];
        data["l"] = x[9];
        data["h"] = x[10];
        data["last_id"] = target.last_id;
        data["bullet_speed"] = cboard.bullet_speed;
        data["d"] = sqrt(x[0] * x[0] + x[2] * x[2] + x[4] * x[4]);
        // data["command_yaw"] = command.yaw * 57.3;
        // data["command_pitch"] = -command.pitch * 57.3;
        }

        cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
        cv::imshow("reprojection", img);

        // 云台响应情况
        // Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

        data["gimbal_yaw"] = ypr[0] * 57.3;
        data["gimbal_pitch"] = -ypr[1] * 57.3;

        auto key = cv::waitKey(10);
        if (key == 'q') break;

        frame_counter++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_fps_time).count();
        if (elapsed >= 1.0) {
        fps = frame_counter / elapsed;
        frame_counter = 0;
        last_fps_time = now;
        }
        plotter.plot(data);
    }
  }

  executor.stop_thread();
  return 0;
}