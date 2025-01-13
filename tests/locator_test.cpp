#include "tasks/radar/locator.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera.hpp"
#include "tasks/radar/arena.hpp"

// 全局变量，用于存储鼠标点击的点
cv::Point2d clicked_point;
bool point_clicked = false;

// 鼠标回调函数
void mouseCallback(int event, int x, int y, int flags, void * userdata)
{
  if (event == cv::EVENT_LBUTTONDOWN) {  // 检测左键点击事件
    clicked_point = cv::Point2d(x, y);
    point_clicked = true;
    std::cout << "Mouse clicked at: (" << x << ", " << y << ")" << std::endl;
  }
}

int main()
{
  // 配置文件路径
  std::string config_path = "configs/radar.yaml";

  // 初始化 Locator 对象
  radar::Locator locator(config_path);

  // 打开视频文件
  cv::VideoCapture cap("records/2024-11-02_00-33-28.avi");

  cv::Mat img;
  const std::string window_name = "Image Window";

  // 创建窗口并设置鼠标回调
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback(window_name, mouseCallback, nullptr);

  locator.self_pos_init({{0, 0}, {0, 10}, {10, 0}, {10, 10}});

  while (true) {
    cap >> img;  // 捕获一帧
    if (img.empty()) break;

    // 如果有鼠标点击，绘制一个小圆点
    if (point_clicked) {
      cv::circle(img, clicked_point, 5, cv::Scalar(0, 0, 255), -1);
      std::vector<Eigen::Vector3d> result = locator.locate({clicked_point});

      if (result.empty()) {
        std::cout << "No result" << std::endl;
      } else {
        std::cout << locator.locate({clicked_point})[0] << std::endl;
      }
      point_clicked = false;  // 重置点击标志
    }

    cv::imshow(window_name, img);  // 显示图像

    if (cv::waitKey(30) == 'q') break;
  }

  cap.release();            // 释放视频捕捉对象
  cv::destroyAllWindows();  // 关闭所有窗口
  return 0;
}
