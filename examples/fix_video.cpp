#include <opencv2/opencv.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明 }"
  "{@input-path    | | 输入视频路径}"
  "{@output-path   | | 输出视频路径}";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;

  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto input_path = cli.get<std::string>(0);
  auto output_path = cli.get<std::string>(1);
  cv::VideoCapture video(input_path);

  auto inited = false;
  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  cv::VideoWriter video_writer;

  for (int i = 0; !exiter.exit(); i++) {
    cv::Mat img;
    video.read(img);
    if (img.empty()) break;

    if (!inited) {
      video_writer = cv::VideoWriter(output_path, fourcc, 30, img.size());
      inited = true;
    }
    video_writer.write(img);

    tools::logger()->info("{}", i);
  }

  video_writer.release();
  return 0;
}