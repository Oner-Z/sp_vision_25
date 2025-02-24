#include "tasks/mono_loc/locator.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera.hpp"
#include "io/ros2/visualizer.hpp"
#include "tasks/mono_loc/arena.hpp"
#include "tasks/mono_loc/robot_detector.hpp"
#include "tools/exiter.hpp"
// 全局变量，用于存储鼠标点击的点
std::vector<cv::Point2f> pnp_img_points;

// 鼠标回调函数
void mouseCallback(int event, int x, int y, int flags, void * userdata)
{
  if (event == cv::EVENT_LBUTTONDOWN) {  // 检测左键点击事件
    pnp_img_points.emplace_back(x, y);
    std::cout << "Mouse clicked at: (" << x << ", " << y << ")" << std::endl;
  }
}

int main()
{
  // 配置文件路径
  std::string config_path = "configs/radar.yaml";

  // 初始化 Locator 对象
  const mono_loc::Arena arena(config_path);
  mono_loc::Locator locator(config_path, arena);
  mono_loc::RobotDetector detector(config_path);
  io::Visualizer visualizer(true);
  tools::Exiter exiter;

  // 打开视频文件
  cv::VideoCapture cap("records/20240516BO301QL_cut.mp4");

  cv::Mat img;
  const std::string window_name = "Image Window";

  // 创建窗口并设置鼠标回调
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback(window_name, mouseCallback, nullptr);

  locator.self_pos_init({{0, 0}, {0, 10}, {10, 0}, {10, 10}});
  bool inited = false;
  visualizer.visualize(arena);
  while (!exiter.exit()) {
    cap >> img;  // 捕获一帧
    if (img.empty()) break;

    if (!inited && pnp_img_points.size() == 4) {
      locator.self_pos_init(pnp_img_points);
      inited = true;
      std::cout << "#####################  inited" << std::endl;
    }
    auto robot_pos_img_list = detector.detect(img);

    std::vector<cv::Point2f> points_img;
    for (auto robot_pos_img : robot_pos_img_list) {
      cv::circle(img, robot_pos_img, 5, {0, 255, 0}, 2);
      points_img.push_back(robot_pos_img);
    }

    auto robot_pos_3d_list = locator.locate(points_img);
    visualizer.visualize(robot_pos_3d_list);

    for (auto & e : robot_pos_3d_list) {
      std::cout << e << std::endl;
    }
    cv::imshow(window_name, img);  // 显示图像

    if (cv::waitKey(1) == 'q') break;
  }

  cap.release();            // 释放视频捕捉对象
  cv::destroyAllWindows();  // 关闭所有窗口
  return 0;
}
