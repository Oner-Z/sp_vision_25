/* high_fps recorder without imshow */
#include <sys/sysinfo.h>  // 避免内存和Swap满导致死机

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }"
  "{fps f          | 140                    | 每秒帧数上限 }"
  "{duration t     | 10                     | 录制时长 }";

struct Frame
{
  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
};

inline bool enoughMemory()
{
  struct sysinfo memInfo;
  sysinfo(&memInfo);

  auto totalMem = memInfo.totalram;
  auto freeMem = memInfo.freeram;
  // tools::logger()->info("Free Mem: {:.2f}", 1.0 * freeMem / totalMem);
  return 1.0 * freeMem / totalMem > 0.05;  // limit 95% physical RAM usage. No swap.
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  auto fps = cli.get<double>("fps");
  auto duration = cli.get<double>("duration");
  io::CBoard & cboard = io::CBoard::get_instance(config_path);
  io::Camera camera(config_path);
  tools::Recorder recorder(fps);

  cv::Mat img;
  Eigen::Quaterniond q;
  // preview before recording start
  while (true) {
    auto stamp = std::chrono::steady_clock::now();
    camera.read(img, stamp);
    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    tools::draw_text(img, "press space to start recording", cv::Point2f(200, 200));
    tools::draw_text(img, "press q to quit", cv::Point2f(200, 250));

    cv::imshow("preview", img);
    char key = cv::waitKey(33);
    if (key == ' ')
      break;
    else if (key == 'q') {
      cv::destroyAllWindows();
      return 0;
    }
  }
  cv::destroyAllWindows();

  // capture to RAM
  std::vector<Frame> frame_buffer;
  auto t_start = std::chrono::steady_clock::now();
  auto t_last = t_start;

  while (tools::delta_time(t_last, t_start) < duration) {
    Frame frame;
    camera.read(frame.img, frame.t);
    frame.q = cboard.imu_at(frame.t - 1ms);
    frame_buffer.emplace_back(frame);
    auto dt = tools::delta_time(frame.t, t_last);
    t_last = frame.t;
    tools::logger()->info("{:.2f} fps", 1 / dt);
    if (!enoughMemory()) {
      tools::logger()->debug("#####  Out of memory! Stopped early.");
      break;
    }
  }

  tools::logger()->info(
    "Captured {} frames in {:.2f} second(s). AVG_FPS = {:.2f}. Now writing to file...",
    frame_buffer.size(), tools::delta_time(t_last, t_start),
    1.0 * frame_buffer.size() / tools::delta_time(t_last, t_start));

  // write to file
  for (size_t i = 0; i < frame_buffer.size(); ++i) {
    auto & frame = frame_buffer[i];
    recorder.record(frame.img, frame.q, frame.t);
    double progress = (static_cast<double>(i + 1) / frame_buffer.size()) * 100;
    if (i % 200 == 0) tools::logger()->info("Writing progress: {:.2f} %", progress);
  }
  tools::logger()->info("Writing to file finished.");

  return 0;
}