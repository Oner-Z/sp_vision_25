#include "io/camera.hpp"
#include "io/ros2/ros2.hpp"
#include "locator.hpp"
#include "tasks/mono_loc/arena.hpp"

int main()
{
  std::string config_path = "config/radar.yaml";
  mono_loc::Arena arena(config_path);

  io::Camera camera(config_path);
  // mono_loc::Detector detector(config_path);
  mono_loc::Locator locator(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  camera.read(img, t);

  std::vector<mono_loc::BBox> robots = detector.detect(img);

  locator.locate(robots);

  return 0;
}