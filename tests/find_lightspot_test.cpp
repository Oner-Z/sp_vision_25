#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/hanging_shooter.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
using namespace hanging_shooting;
int main(int argc, char * argv[])  //
{
  auto yaml = YAML::LoadFile("configs/example.yaml");
  auto img_num = yaml["img_num"].as<std::string>();
  std::string img_path = "./assets/img/" + img_num + ".jpg";
  cv::Mat img;
  img = cv::imread(img_path);
  HangingShooter hangingshooter;

  std::vector<LightSpot> lightspots = hangingshooter.detect(img);
  cv::Mat drawlightspot = img.clone();
  for (const auto & lightspot : lightspots) {
    tools::draw_points(drawlightspot, lightspot.contour);
    cv::Mat tvec, rvec;
    hangingshooter.solvePnP(lightspot, tvec, rvec);
    tools::draw_text(
      drawlightspot,
      fmt::format(
        "tvec:  x{: .2f} y{: .2f} z{: .2f}", tvec.at<double>(0), tvec.at<double>(1),
        tvec.at<double>(2)),
      cv::Point2f(10, 60), cv::Scalar(0, 255, 255));
    tools::draw_text(
      drawlightspot,
      fmt::format(
        "rvec:  x{: .2f} y{: .2f} z{: .2f}", rvec.at<double>(0), rvec.at<double>(1),
        rvec.at<double>(2)),
      cv::Point2f(10, 260), cv::Scalar(0, 255, 255));
  }
  std::cout << lightspots.size() << std::endl;
  cv::resize(drawlightspot, drawlightspot, {}, 0.5, 0.5);

  cv::imshow("lightspot", drawlightspot);
  cv::waitKey(0);
  return 0;
}
