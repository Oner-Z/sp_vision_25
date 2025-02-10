#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"

const std::string keys =
  "{help h usage ?  |                     | 输出命令行参数说明}"
  "{can             | can0                | can端口名称     }"
  "{config-path c   | configs/camera.yaml | yaml配置文件路径 }"
  "{output-folder o | assets/video        | 输出文件夹路径   }";

// 处理命令行参数
void parseCommandLine(
  int argc, char ** argv, std::string & config_path, std::string & can, std::string & output_folder)
{
  cv::CommandLineParser parser(argc, argv, keys);
  if (parser.has("help")) {
    parser.printMessage();
    exit(0);
  }

  config_path = parser.get<std::string>("config-path");
  can = parser.get<std::string>("can");
  output_folder = parser.get<std::string>("output-folder");
}

// 捕获视频并保存为文件
void capture_video(
  const std::string & config_path, const std::string & can, const std::string & output_folder)
{
  io::Camera camera(config_path);  // 这个对象需要根据你的实际相机接口库进行配置
  cv::Mat frame;
  std::chrono::steady_clock::time_point timestamp;

  std::string video_output_path = output_folder + "/output_video.avi";
  camera.read(frame, timestamp);
  int frame_width = frame.size().width;
  int frame_height = frame.size().height;

  // 定义视频编码器和视频文件
  cv::VideoWriter video_writer(
    video_output_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
    cv::Size(frame_width, frame_height));

  if (!video_writer.isOpened()) {
    std::cerr << "无法打开视频文件进行写入!" << std::endl;
    return;
  }

  std::cout << "开始捕获视频..." << std::endl;

  while (true) {
    // 从相机捕获一帧
    camera.read(frame, timestamp);

    // 如果帧为空，说明视频流结束或发生错误，退出循环
    if (frame.empty()) {
      std::cerr << "捕获失败，视频结束或发生错误!" << std::endl;
      break;
    }

    // 显示捕获的帧（可选）
    cv::imshow("Captured Frame", frame);

    // 将捕获的帧写入视频文件
    video_writer.write(frame);

    // 按下 'q' 键退出
    if (cv::waitKey(1) == 'q') {
      break;
    }
  }

  // 释放资源
  video_writer.release();
  cv::destroyAllWindows();
}

int main(int argc, char ** argv)
{
  std::string config_path, can, output_folder;
  parseCommandLine(argc, argv, config_path, can, output_folder);
  std::filesystem::create_directories(output_folder);
  capture_video(config_path, can, output_folder);
  return 0;
}