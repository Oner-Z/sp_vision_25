#include "yolov8kp.hpp"
namespace auto_buff
{
YOLOV8KP::YOLOV8KP(const std::string & config)
{
  auto yaml = YAML::LoadFile(config);
  std::string model_path = yaml["model"].as<std::string>();
  model = core.read_model(model_path);
  // printInputAndOutputsInfo(*model); // 打印模型信息
  /// 载入并编译模型
  compiled_model = core.compile_model(model, "AUTO");
  /// 创建推理请求
  infer_request = compiled_model.create_infer_request();
  // 获取模型输入节点
  input_tensor = infer_request.get_input_tensor();
}
std::vector<YOLOV8KP::Object> YOLOV8KP::work(cv::Mat & image)
{
  /// 设置模型输入
  const int64 start = cv::getTickCount();
  // 读取图片并按照模型输入要求进行预处理
  const float factor = fill_tensor_data_image(input_tensor, image);
  /// 执行推理计算
  infer_request.infer();
  /// 处理推理计算结果
  // 获得推理结果
  const ov::Tensor output = infer_request.get_output_tensor();
  const ov::Shape output_shape = output.get_shape();
  const float * output_buffer = output.data<const float>();
  // 解析推理结果
  const int out_rows = output_shape[1];  // 获得"output"节点的rows
  const int out_cols = output_shape[2];  // 获得"output"节点的cols
  const cv::Mat det_output(out_rows, out_cols, CV_32F, (float *)output_buffer);

  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<std::vector<float>> objects_keypoints;
  // 输出格式是[15,8400], 每列代表一个框(即最多有8400个框), 前面4行分别是[cx, cy, ow, oh], 中间score, 最后5*2关键点(3代表每个关键点的信息, 包括[x, y, visibility])
  // 15=4+1+5*2      56
  for (int i = 0; i < det_output.cols; ++i) {
    const float score = det_output.at<float>(4, i);
    // 置信度 0～1之间
    if (score > 0.3f) {
      const float cx = det_output.at<float>(0, i);
      const float cy = det_output.at<float>(1, i);
      const float ow = det_output.at<float>(2, i);
      const float oh = det_output.at<float>(3, i);
      cv::Rect box;
      box.x = static_cast<int>((cx - 0.5 * ow) * factor);
      box.y = static_cast<int>((cy - 0.5 * oh) * factor);
      box.width = static_cast<int>(ow * factor);
      box.height = static_cast<int>(oh * factor);

      boxes.push_back(box);
      confidences.push_back(score);

      // 获取关键点
      std::vector<float> keypoints;
      cv::Mat kpts = det_output.col(i).rowRange(5, 15);
      for (int j = 0; j < 5; ++j) {
        const float x = kpts.at<float>(j * 2 + 0, 0) * factor;
        const float y = kpts.at<float>(j * 2 + 1, 0) * factor;
        // const float s = kpts.at<float>(j * 3 + 2, 0);
        keypoints.push_back(x);
        keypoints.push_back(y);
        // keypoints.push_back(s);
      }
      objects_keypoints.push_back(keypoints);
    }
  }

  const int radius = 2;
  const cv::Size & shape = image.size();
  std::vector<cv::Scalar> limbColorPalette;
  std::vector<cv::Scalar> kptColorPalette;
  for (int index : limbColorIndices) limbColorPalette.push_back(posePalette[index]);
  for (int index : kptColorIndices) kptColorPalette.push_back(posePalette[index]);

  // NMS, 消除具有较低置信度的冗余重叠框
  std::vector<int> indexes;
  cv::dnn::NMSBoxes(boxes, confidences, 0.25f, 0.45f, indexes);
  std::vector<Object> object_result;
  for (size_t i = 0; i < indexes.size(); ++i) {
    Object obj;
    const int index = indexes[i];
    obj.rect = boxes[index];
    obj.prob = confidences[index];
    /// 绘制关键点和连线
    const std::vector<float> & keypoint = objects_keypoints[index];
    // 绘制关键点
    for (int i = 0; i < 5; ++i) {
      const int idx = i * 2;
      const int x_coord = static_cast<int>(keypoint[idx]);
      const int y_coord = static_cast<int>(keypoint[idx + 1]);
      obj.kpt.push_back(cv::Point2f(x_coord, y_coord));
    }
    object_result.push_back(obj);

    //             // 绘制矩形框
    cv::rectangle(image, obj.rect, cv::Scalar(0, 0, 255), 2, 8);
    //             // 绘制标签
    //             const std::string label = "Person:" + std::to_string(confidences[index]).substr(0, 4);
    //             const cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
    //             const cv::Rect textBox(obj.rect.tl().x, obj.rect.tl().y - 15, textSize.width, textSize.height + 5);
    //             cv::rectangle(image, textBox, cv::Scalar(0, 255, 255), cv::FILLED);
    //             cv::putText(image, label, cv::Point(obj.rect.tl().x, obj.rect.tl().y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
    //                         cv::Scalar(0, 0, 0));
    //             // 绘制关键点
    //             for (int i = 0; i < 5; ++i)
    //             {
    //                 const int idx = i * 2;
    //                 const int x_coord = static_cast<int>(keypoint[idx]);
    //                 const int y_coord = static_cast<int>(keypoint[idx + 1]);
    //                 if ((x_coord % shape.width) != 0 && (y_coord % shape.height) != 0)
    //                     cv::circle(image, cv::Point(x_coord, y_coord), radius, kptColorPalette[i], -1, cv::LINE_AA);
    //             }
  }
  /// 计算FPS
  const float t = (cv::getTickCount() - start) / static_cast<float>(cv::getTickFrequency());
  cv::putText(
    image, cv::format("FPS: %.2f", 1.0 / t), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0,
    cv::Scalar(255, 0, 0), 2, 8);

  // #ifdef SAVE
  //         /// 保存结果图
  //         save("save", image);
  // #endif
  return object_result;
}

void YOLOV8KP::convert(
  const cv::Mat & input, cv::Mat & output, const bool normalize, const bool exchangeRB)
{
  input.convertTo(output, CV_32F);
  if (normalize) output = output / 255.0;  // 归一化到[0, 1]
  if (exchangeRB) cv::cvtColor(output, output, cv::COLOR_BGR2RGB);
}

float YOLOV8KP::fill_tensor_data_image(ov::Tensor & input_tensor, const cv::Mat & input_image)
{
  /// letterbox变换: 不改变宽高比(aspect ratio), 将input_image缩放并放置到blob_image左上角
  const ov::Shape tensor_shape = input_tensor.get_shape();
  const size_t num_channels = tensor_shape[1];
  const size_t height = tensor_shape[2];
  const size_t width = tensor_shape[3];
  // 缩放因子
  const float scale = std::min(height / float(input_image.rows), width / float(input_image.cols));
  const cv::Matx23f matrix{
    scale, 0.0, 0.0, 0.0, scale, 0.0,
  };
  cv::Mat blob_image;
  // 下面根据scale范围进行数据转换, 这只是为了提高一点速度(主要是提高了交换通道的速度)
  // 如果不在意这点速度提升的可以固定一种做法(两个if分支随便一个都可以)
  if (scale < 1.0f) {
    // 要缩小, 那么先缩小再交换通道
    cv::warpAffine(input_image, blob_image, matrix, cv::Size(width, height));
    convert(blob_image, blob_image, true, true);
  } else {
    // 要放大, 那么先交换通道再放大
    convert(input_image, blob_image, true, true);
    cv::warpAffine(blob_image, blob_image, matrix, cv::Size(width, height));
  }

  /// 将图像数据填入input_tensor
  float * const input_tensor_data = input_tensor.data<float>();
  // 原有图片数据为 HWC格式，模型输入节点要求的为 CHW 格式
  for (size_t c = 0; c < num_channels; c++) {
    for (size_t h = 0; h < height; h++) {
      for (size_t w = 0; w < width; w++) {
        input_tensor_data[c * width * height + h * width + w] =
          blob_image.at<cv::Vec<float, 3>>(h, w)[c];
      }
    }
  }
  return 1 / scale;
}

void YOLOV8KP::printInputAndOutputsInfo(const ov::Model & network)
{
  std::cout << "model name: " << network.get_friendly_name() << std::endl;

  const std::vector<ov::Output<const ov::Node>> inputs = network.inputs();
  for (const ov::Output<const ov::Node> & input : inputs) {
    std::cout << "    inputs" << std::endl;

    const std::string name = input.get_names().empty() ? "NONE" : input.get_any_name();
    std::cout << "        input name: " << name << std::endl;

    const ov::element::Type type = input.get_element_type();
    std::cout << "        input type: " << type << std::endl;

    const ov::Shape shape = input.get_shape();
    std::cout << "        input shape: " << shape << std::endl;
  }

  const std::vector<ov::Output<const ov::Node>> outputs = network.outputs();
  for (const ov::Output<const ov::Node> & output : outputs) {
    std::cout << "    outputs" << std::endl;

    const std::string name = output.get_names().empty() ? "NONE" : output.get_any_name();
    std::cout << "        output name: " << name << std::endl;

    const ov::element::Type type = output.get_element_type();
    std::cout << "        output type: " << type << std::endl;

    const ov::Shape shape = output.get_shape();
    std::cout << "        output shape: " << shape << std::endl;
  }
}

void YOLOV8KP::save(const std::string & programName, const cv::Mat & image)
{
  const std::filesystem::path saveDir = "../result/";
  if (!std::filesystem::exists(saveDir)) {
    std::filesystem::create_directories(saveDir);
  }
  const std::filesystem::path savePath = saveDir / (programName + ".jpg");
  cv::imwrite(savePath.string(), image);
}
}  // namespace auto_buff