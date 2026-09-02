#include "armor_detector/armor_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <openvino/openvino.hpp>

#include "armor_detector/armor_utils.hpp"

namespace armor_detector
{

namespace
{

constexpr int kInputWidth = 640;
constexpr int kInputHeight = 640;
constexpr int kColorClassCount = 4;
constexpr int kRobotClassCount = 9;
constexpr int kOutputDimension = 8 + 1 + kColorClassCount + kRobotClassCount;

double sigmoid(double value)
{
  if (value > 0.0) {
    return 1.0 / (1.0 + std::exp(-value));
  }
  const auto exponential = std::exp(value);
  return exponential / (1.0 + exponential);
}

cv::Rect boundingRect(const std::array<cv::Point2f, 4> & corners)
{
  return cv::boundingRect(std::vector<cv::Point2f>(corners.begin(), corners.end()));
}

}  // namespace

class ArmorDetector::Impl
{
public:
  explicit Impl(const ArmorDetectorConfig & config)
  : confidence_threshold(config.confidence_threshold), nms_threshold(config.nms_threshold)
  {
    if (config.model_path.empty()) {
      throw std::invalid_argument("装甲板模型路径不能为空");
    }
    if (confidence_threshold < 0.0F || confidence_threshold > 1.0F) {
      throw std::invalid_argument("confidence_threshold 必须位于 [0, 1]");
    }
    if (nms_threshold < 0.0F || nms_threshold > 1.0F) {
      throw std::invalid_argument("nms_threshold 必须位于 [0, 1]");
    }

    model = core.read_model(config.model_path);
    ov::preprocess::PrePostProcessor preprocessor(model);
    preprocessor.input().tensor()
      .set_element_type(ov::element::u8)
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::BGR);
    preprocessor.input().preprocess()
      .convert_element_type(ov::element::f32)
      .convert_color(ov::preprocess::ColorFormat::RGB)
      .scale({255.0, 255.0, 255.0});
    preprocessor.input().model().set_layout("NCHW");
    preprocessor.output().tensor().set_element_type(ov::element::f32);
    model = preprocessor.build();
    compiled_model = core.compile_model(
      model, config.device_name, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    infer_request = compiled_model.create_infer_request();
  }

  std::vector<ArmorDetection> detect(const cv::Mat & image, int target_color)
  {
    if (image.empty()) {
      return {};
    }
    if (image.type() != CV_8UC3) {
      throw std::invalid_argument("装甲板检测器只接受 bgr8 图像");
    }

    cv::Mat resized_image;
    cv::resize(image, resized_image, cv::Size(kInputWidth, kInputHeight));
    const ov::Shape input_shape = {1U, kInputHeight, kInputWidth, 3U};
    ov::Tensor input_tensor(ov::element::u8, input_shape, resized_image.data);
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    auto output_tensor = infer_request.get_output_tensor(0);
    const auto output_shape = output_tensor.get_shape();
    if (output_shape.size() != 3U || static_cast<int>(output_shape[2]) != kOutputDimension) {
      throw std::runtime_error("0526 装甲板模型输出维度不匹配");
    }

    const auto anchor_count = static_cast<int>(output_shape[1]);
    cv::Mat output(anchor_count, kOutputDimension, CV_32F, output_tensor.data<float>());
    const auto scale_x = static_cast<float>(image.cols) / kInputWidth;
    const auto scale_y = static_cast<float>(image.rows) / kInputHeight;

    std::vector<ArmorDetection> candidates;
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    candidates.reserve(anchor_count);

    for (int anchor = 0; anchor < anchor_count; ++anchor) {
      const auto confidence = static_cast<float>(sigmoid(output.at<float>(anchor, 8)));
      if (confidence < confidence_threshold) {
        continue;
      }

      cv::Point color_index;
      cv::minMaxLoc(output.row(anchor).colRange(9, 13), nullptr, nullptr, nullptr, &color_index);
      if (!shouldKeepColor(color_index.x, target_color)) {
        continue;
      }

      cv::Point class_index;
      cv::minMaxLoc(output.row(anchor).colRange(13, 22), nullptr, nullptr, nullptr, &class_index);
      const auto id = mapModelClass2HeroId(class_index.x);
      if (id < 0) {
        continue;
      }

      ArmorDetection detection;
      detection.corners = {
        cv::Point2f(output.at<float>(anchor, 0) * scale_x, output.at<float>(anchor, 1) * scale_y),
        cv::Point2f(output.at<float>(anchor, 6) * scale_x, output.at<float>(anchor, 7) * scale_y),
        cv::Point2f(output.at<float>(anchor, 4) * scale_x, output.at<float>(anchor, 5) * scale_y),
        cv::Point2f(output.at<float>(anchor, 2) * scale_x, output.at<float>(anchor, 3) * scale_y)};
      const auto rect = boundingRect(detection.corners);
      const auto width = static_cast<float>(rect.width);
      const auto height = static_cast<float>(rect.height);
      const auto aspect_ratio = std::max(width, height) / std::max(1.0F, std::min(width, height));
      if (aspect_ratio < 1.0F || aspect_ratio > 6.0F) {
        continue;
      }
      detection.id = id;
      detection.color = color_index.x;
      detection.confidence = confidence;
      candidates.push_back(detection);
      boxes.push_back(rect);
      confidences.push_back(confidence);
    }

    std::vector<int> kept_indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold, nms_threshold, kept_indices);
    std::vector<ArmorDetection> results;
    results.reserve(kept_indices.size());
    for (const auto index : kept_indices) {
      if (index >= 0 && index < static_cast<int>(candidates.size())) {
        results.push_back(std::move(candidates[static_cast<std::size_t>(index)]));
      }
    }
    return results;
  }

private:
  float confidence_threshold;
  float nms_threshold;
  ov::Core core;
  std::shared_ptr<ov::Model> model;
  ov::CompiledModel compiled_model;
  ov::InferRequest infer_request;
};

ArmorDetector::ArmorDetector(const ArmorDetectorConfig & config)
: impl_(std::make_unique<Impl>(config))
{
}

ArmorDetector::~ArmorDetector() = default;

std::vector<ArmorDetection> ArmorDetector::detect(const cv::Mat & image, int target_color)
{
  return impl_->detect(image, target_color);
}

}  // armor_detector
