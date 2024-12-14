#ifndef COMMOM_HPP
#define COMMOM_HPP
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

namespace auto_buff
{
// 定义skeleton的连接关系以及color mappings
// skeleton[0]={16, 14}表示objects_keypoints[16]和objects_keypoints[14]连一条线, posePalette[limbColorIndices[0]]表示这条线的颜色
static const std::vector<std::vector<int>> skeleton = {
  {16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13}, {6, 12}, {7, 13}, {6, 7}, {6, 8}, {7, 9},
  {8, 10},  {9, 11},  {2, 3},   {1, 2},   {1, 3},   {2, 4},  {3, 5},  {4, 6}, {5, 7}};
static const std::vector<cv::Scalar> posePalette = {
  cv::Scalar(255, 128, 0),   cv::Scalar(255, 153, 51),  cv::Scalar(255, 178, 102),
  cv::Scalar(230, 230, 0),   cv::Scalar(255, 153, 255), cv::Scalar(153, 204, 255),
  cv::Scalar(255, 102, 255), cv::Scalar(255, 51, 255),  cv::Scalar(102, 178, 255),
  cv::Scalar(51, 153, 255),  cv::Scalar(255, 153, 153), cv::Scalar(255, 102, 102),
  cv::Scalar(255, 51, 51),   cv::Scalar(153, 255, 153), cv::Scalar(102, 255, 102),
  cv::Scalar(51, 255, 51),   cv::Scalar(0, 255, 0),     cv::Scalar(0, 0, 255),
  cv::Scalar(255, 0, 0),     cv::Scalar(255, 255, 255)};
static const std::vector<int> limbColorIndices = {9, 9, 9,  9,  7,  7,  7,  0,  0, 0,
                                                  0, 0, 16, 16, 16, 16, 16, 16, 16};
static const std::vector<int> kptColorIndices = {16, 16, 16, 16, 16, 0, 0, 0, 0,
                                                 0,  0,  9,  9,  9,  9, 9, 9};
const std::vector<std::string> class_names = {"buff", "r"};

class YOLOV8KP
{
public:
  struct Object
  {
    cv::Rect_<float> rect;
    int label;
    float prob;
    std::vector<cv::Point2f> kpt;
  };

  YOLOV8KP(const std::string & config);

  std::vector<Object> work(cv::Mat & image);

private:
  ov::Core core;  // 创建OpenVINO Runtime Core对象
  std::shared_ptr<ov::Model> model;
  ov::CompiledModel compiled_model;
  ov::InferRequest infer_request;
  ov::Tensor input_tensor;
  /// 转换图像数据: 先转换元素类型, (可选)然后归一化到[0, 1], (可选)然后交换RB通道
  void convert(
    const cv::Mat & input, cv::Mat & output, const bool normalize, const bool exchangeRB);

  /*!
     * \brief fill_tensor_data_image 对网络的输入为图片数据的节点进行赋值，实现图片数据输入网络
     * \param input_tensor 输入节点的tensor
     * \param input_image 输入图片的数据
     * \return 缩放因子, 该缩放是为了将input_image塞进input_tensor
     */
  float fill_tensor_data_image(ov::Tensor & input_tensor, const cv::Mat & input_image);

  // 打印模型信息, 这个函数修改自$${OPENVINO_COMMON}/utils/src/args_helper.cpp的同名函数
  void printInputAndOutputsInfo(const ov::Model & network);

  // 将image保存为"../result/$${programName}.jpg"
  void save(const std::string & programName, const cv::Mat & image);
};
}  // namespace auto_buff
#endif