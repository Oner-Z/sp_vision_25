#include "io/camera.hpp"
#include "locator.hpp"
#include "tasks/radar/arena.hpp"

int main()
{
  std::string config_path = "config/radar.yaml";

  io::Camera camera(config_path);
  // radar::Detector detector(config_path);
  radar::Locator locator(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  camera.read(img, t);

  std::vector<radar::BBox> robots = detector.detect(img);

  locator.locate(robots);

  radar::Arena arena(config_path);

  return 0;
}