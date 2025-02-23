#ifndef MONO_LOC__ROBOT_DETECTOR_HPP
#define MONO_LOC__ROBOT_DETECTOR_HPP

#include <yaml-cpp/yaml.h>

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "tools/logger.hpp"

namespace mono_loc
{

class RobotDetector
{
public:
  RobotDetector(const std::string & config_path, bool debug = true)
  {
    auto yaml = YAML::LoadFile(config_path);

    model_path_ = yaml["robot_model_path"].as<std::string>();
    device_ = yaml["device"].as<std::string>();
    min_confidence_ = yaml["min_confidence"].as<double>();

    auto model = core_.read_model(model_path_);
    ov::preprocess::PrePostProcessor ppp(model);
    auto & input = ppp.input();

    input.tensor()
      .set_element_type(ov::element::u8)
      .set_shape({1, 640, 640, 3})
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::BGR);

    input.model().set_layout("NCHW");

    input.preprocess()
      .convert_element_type(ov::element::f32)
      .convert_color(ov::preprocess::ColorFormat::RGB)
      .scale(255.0);

    model = ppp.build();
    compiled_model_ = core_.compile_model(
      model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  }

  std::list<cv::Point2f> detect(const cv::Mat & bgr_img, int frame_count = -1)
  {
    if (bgr_img.empty()) {
      tools::logger()->warn("Empty img!, camera drop!");
      return std::list<cv::Point2f>();
    }

    auto x_scale = static_cast<double>(640) / bgr_img.rows;
    auto y_scale = static_cast<double>(640) / bgr_img.cols;
    auto scale = std::min(x_scale, y_scale);
    auto h = static_cast<int>(bgr_img.rows * scale);
    auto w = static_cast<int>(bgr_img.cols * scale);

    // preproces
    auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    auto roi = cv::Rect(0, 0, w, h);
    cv::resize(bgr_img, input(roi), {w, h});
    ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

    /// infer
    auto infer_request = compiled_model_.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    // postprocess
    auto output_tensor = infer_request.get_output_tensor();
    auto output_shape = output_tensor.get_shape();
    cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

    // std::cout << '#' << output_shape << std::endl;
    // std::cout << '*' << output.size() << std::endl;
    cv::transpose(output, output);

    std::vector<int> ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> armors_key_points;
    for (int r = 0; r < output.rows; r++) {
      auto xywh = output.row(r).colRange(0, 4);
      auto scores = output.row(r).colRange(4, 4 + class_num_);

      double score;
      cv::Point max_point;
      cv::minMaxLoc(scores, nullptr, &score, nullptr, &max_point);

      if (score < score_threshold_) continue;

      auto x = xywh.at<float>(0);
      auto y = xywh.at<float>(1);
      auto w = xywh.at<float>(2);
      auto h = xywh.at<float>(3);
      auto left = static_cast<int>((x - 0.5 * w) / scale);
      auto top = static_cast<int>((y - 0.5 * h) / scale);
      auto width = static_cast<int>(w / scale);
      auto height = static_cast<int>(h / scale);

      ids.emplace_back(max_point.x);
      confidences.emplace_back(score);
      boxes.emplace_back(left, top, width, height);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

    std::list<cv::Point2f> ret;
    for (auto box : boxes) {
      // cv::rectangle(bgr_img, box, {0, 255, 0}, 2);
      cv::Point2f bottom_center = {box.x + box.width / 2, box.y + box.height};
      ret.push_back(bottom_center);
    }
    return ret;
  }

private:
  std::string device_, model_path_;
  bool debug_;

  const int class_num_ = 1;
  const float nms_threshold_ = 0.3;
  const float score_threshold_ = 0.7;
  double min_confidence_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;

  cv::Rect roi_;
  cv::Point2f offset_;
};

}  // namespace mono_loc

#endif