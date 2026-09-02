#include "armor_detector/armor_detector_node.hpp"

#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#else
  #include <cv_bridge/cv_bridge.h>
#endif
#include <geometry_msgs/msg/point32.hpp>

#include "armor_detector/armor_detector.hpp"

namespace armor_detector
{

namespace
{

rclcpp::QoS highRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

std::string resolveModelPath(const std::string & configured_path)
{
  if (configured_path.empty()) {
    throw std::invalid_argument("model_path 不能为空");
  }
  const std::filesystem::path path(configured_path);
  if (path.is_absolute()) {
    return path.string();
  }
  const std::filesystem::path package_share(
    ament_index_cpp::get_package_share_directory("armor_detector"));
  const auto package_model = package_share / path;
  if (std::filesystem::exists(package_model)) {
    return package_model.string();
  }

  // 模型统一存放在 heros/model。开发环境与部署目录均从功能包 share 目录向上查找。
  for (auto directory = package_share; !directory.empty(); directory = directory.parent_path()) {
    const auto project_model = directory / "model" / path.filename();
    if (std::filesystem::exists(project_model)) {
      return project_model.string();
    }
  }
  return package_model.string();
}

}  // namespace

DetectorNode::DetectorNode()
: Node("armor_detector_node")
{
  declare_parameter<std::string>("image_topic", "/hero/camera/selected/image_raw");
  declare_parameter<std::string>("gimbal_state_topic", "/hero/gimbal/state");
  declare_parameter<std::string>("armor_topic", "/hero/detector/armors");
  declare_parameter<std::string>("model_path", "model/0526.onnx");
  declare_parameter<std::string>("inference_device", "CPU");
  declare_parameter<double>("confidence_threshold", 0.75);
  declare_parameter<double>("nms_threshold", 0.45);
  // -1：根据 robot_color 自动选择敌方颜色；-2：不按颜色过滤；0/1：固定目标颜色。
  declare_parameter<int>("target_color", -1);

  target_color_ = get_parameter("target_color").as_int();
  if (target_color_ != -2 && target_color_ != -1 && target_color_ != 0 && target_color_ != 1) {
    throw std::invalid_argument("target_color 只能是 -2、-1、0 或 1");
  }

  ArmorDetectorConfig detector_config;
  detector_config.model_path = resolveModelPath(get_parameter("model_path").as_string());
  detector_config.device_name = get_parameter("inference_device").as_string();
  detector_config.confidence_threshold =
    static_cast<float>(get_parameter("confidence_threshold").as_double());
  detector_config.nms_threshold = static_cast<float>(get_parameter("nms_threshold").as_double());
  armor_detector_ = std::make_unique<ArmorDetector>(detector_config);

  const auto image_topic = get_parameter("image_topic").as_string();
  const auto armor_topic = get_parameter("armor_topic").as_string();
  if (image_topic.empty() || armor_topic.empty()) {
    throw std::invalid_argument("图像与检测结果话题不能为空");
  }
  armor_pub_ = create_publisher<hero_msgs::msg::ArmorArray>(armor_topic, highRateQos());
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic, highRateQos(),
    [this](const sensor_msgs::msg::Image::ConstSharedPtr image) { receiveImage(image); });
  if (target_color_ == -1) {
    gimbal_state_sub_ = create_subscription<hero_msgs::msg::GimbalState>(
      get_parameter("gimbal_state_topic").as_string(), highRateQos(),
      [this](const hero_msgs::msg::GimbalState::ConstSharedPtr state) { receiveGimbalState(state); });
  }

  running_.store(true);
  worker_ = std::thread([this]() { processLoop(); });
  RCLCPP_INFO(
    get_logger(), "装甲板检测节点已启动：输入 %s，输出 %s，颜色模式 %d", image_topic.c_str(),
    armor_topic.c_str(), target_color_);
}

DetectorNode::~DetectorNode()
{
  running_.store(false);
  image_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void DetectorNode::receiveImage(const sensor_msgs::msg::Image::ConstSharedPtr & image)
{
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image_ = image;
  }
  image_cv_.notify_one();
}

void DetectorNode::receiveGimbalState(const hero_msgs::msg::GimbalState::ConstSharedPtr & state)
{
  if (state->robot_color == 0U || state->robot_color == 1U) {
    robot_color_.store(static_cast<int>(state->robot_color));
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "忽略未知 robot_color：%u",
      static_cast<unsigned int>(state->robot_color));
  }
}

void DetectorNode::processLoop()
{
  while (rclcpp::ok() && running_.load()) {
    sensor_msgs::msg::Image::ConstSharedPtr image;
    {
      std::unique_lock<std::mutex> lock(image_mutex_);
      image_cv_.wait(lock, [this]() { return !running_.load() || latest_image_ != nullptr; });
      if (!running_.load()) {
        return;
      }
      image = std::move(latest_image_);
      latest_image_.reset();
    }

    const auto target_color = resolveTargetColor();
    if (target_color == -1) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "尚未收到有效 robot_color，发布空检测结果");
      publishDetections(image, {});
      continue;
    }

    try {
      const auto cv_image = cv_bridge::toCvShare(image, "bgr8");
      publishDetections(image, armor_detector_->detect(cv_image->image, target_color));
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "装甲板检测失败：%s", error.what());
      publishDetections(image, {});
    }
  }
}

void DetectorNode::publishDetections(
  const sensor_msgs::msg::Image::ConstSharedPtr & image,
  const std::vector<ArmorDetection> & detections)
{
  hero_msgs::msg::ArmorArray output;
  output.header = image->header;
  output.armors.reserve(detections.size());
  for (const auto & detection : detections) {
    hero_msgs::msg::Armor armor;
    armor.header = image->header;
    for (std::size_t index = 0; index < detection.corners.size(); ++index) {
      auto & corner = armor.corners[index];
      corner.x = detection.corners[index].x;
      corner.y = detection.corners[index].y;
      corner.z = 0.0F;
    }
    armor.id = static_cast<std::uint8_t>(detection.id);
    armor.color = static_cast<std::uint8_t>(detection.color);
    armor.confidence = detection.confidence;
    output.armors.push_back(std::move(armor));
  }
  armor_pub_->publish(std::move(output));
}

int DetectorNode::resolveTargetColor() const
{
  if (target_color_ != -1) {
    return target_color_;
  }
  const auto robot_color = robot_color_.load();
  if (robot_color == 0) {
    return 1;
  }
  if (robot_color == 1) {
    return 0;
  }
  return -1;
}

}  // armor_detector
