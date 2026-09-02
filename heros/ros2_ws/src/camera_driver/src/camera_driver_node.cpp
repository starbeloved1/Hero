#include "camera_driver/camera_driver_node.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <thread>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "camera_driver/daheng_camera.hpp"

namespace camera_driver
{

namespace
{

rclcpp::QoS highRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

}  // namespace

struct CameraDriverNode::Stream
{
  std::string name;
  std::string frame_id;
  DahengCamera camera;
  sensor_msgs::msg::CameraInfo camera_info;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
  std::thread thread;
};

CameraDriverNode::CameraDriverNode()
: Node("camera_driver_node")
{
  declare_parameter<bool>("aim8mm.enabled", true);
  declare_parameter<bool>("base.enabled", true);
  for (const auto & name : {"aim8mm", "base"}) {
    declare_parameter<std::string>(std::string(name) + ".serial_number", "");
    declare_parameter<std::string>(std::string(name) + ".frame_id", std::string(name) + "_camera_optical_frame");
    declare_parameter<std::string>(
      std::string(name) + ".image_topic", std::string("/hero/camera/") + name + "/image_raw");
    declare_parameter<std::string>(
      std::string(name) + ".camera_info_topic", std::string("/hero/camera/") + name + "/camera_info");
    declare_parameter<int>(std::string(name) + ".width", 1280);
    declare_parameter<int>(std::string(name) + ".height", 1024);
    declare_parameter<double>(std::string(name) + ".exposure_time_us", name == std::string("base") ? 12000.0 : 4000.0);
    declare_parameter<double>(std::string(name) + ".gain", name == std::string("base") ? 13.0 : 16.0);
    declare_parameter<double>(std::string(name) + ".fx", 0.0);
    declare_parameter<double>(std::string(name) + ".fy", 0.0);
    declare_parameter<double>(std::string(name) + ".cx", 0.0);
    declare_parameter<double>(std::string(name) + ".cy", 0.0);
    declare_parameter<double>(std::string(name) + ".k1", 0.0);
    declare_parameter<double>(std::string(name) + ".k2", 0.0);
    declare_parameter<double>(std::string(name) + ".p1", 0.0);
    declare_parameter<double>(std::string(name) + ".p2", 0.0);
    declare_parameter<double>(std::string(name) + ".k3", 0.0);
  }

  for (const auto & name : {"aim8mm", "base"}) {
    if (!get_parameter(std::string(name) + ".enabled").as_bool()) {
      continue;
    }
    auto stream = std::make_unique<Stream>();
    stream->name = name;
    stream->frame_id = get_parameter(std::string(name) + ".frame_id").as_string();
    DahengCameraConfig config;
    config.serial_number = get_parameter(std::string(name) + ".serial_number").as_string();
    config.width = get_parameter(std::string(name) + ".width").as_int();
    config.height = get_parameter(std::string(name) + ".height").as_int();
    config.exposure_time_us = get_parameter(std::string(name) + ".exposure_time_us").as_double();
    config.gain = get_parameter(std::string(name) + ".gain").as_double();
    std::string error;
    if (!stream->camera.open(config, error)) {
      throw std::runtime_error(name + std::string(" 相机打开失败：") + error);
    }
    const auto image_topic = get_parameter(std::string(name) + ".image_topic").as_string();
    const auto camera_info_topic = get_parameter(std::string(name) + ".camera_info_topic").as_string();
    if (image_topic.empty() || camera_info_topic.empty()) {
      throw std::runtime_error(name + std::string(" 相机话题不能为空"));
    }
    stream->image_pub = create_publisher<sensor_msgs::msg::Image>(image_topic, highRateQos());
    stream->camera_info_pub =
      create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, highRateQos());
    stream->camera_info.width = static_cast<std::uint32_t>(config.width);
    stream->camera_info.height = static_cast<std::uint32_t>(config.height);
    stream->camera_info.distortion_model = "plumb_bob";
    const auto fx = get_parameter(std::string(name) + ".fx").as_double();
    const auto fy = get_parameter(std::string(name) + ".fy").as_double();
    const auto cx = get_parameter(std::string(name) + ".cx").as_double();
    const auto cy = get_parameter(std::string(name) + ".cy").as_double();
    stream->camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    stream->camera_info.d = {
      get_parameter(std::string(name) + ".k1").as_double(),
      get_parameter(std::string(name) + ".k2").as_double(),
      get_parameter(std::string(name) + ".p1").as_double(),
      get_parameter(std::string(name) + ".p2").as_double(),
      get_parameter(std::string(name) + ".k3").as_double()};
    stream->camera_info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    streams_.push_back(std::move(stream));
  }
  if (streams_.empty()) {
    throw std::runtime_error("至少启用一路相机");
  }
  running_.store(true);
  for (auto & stream : streams_) {
    stream->thread = std::thread([this, raw_stream = stream.get()]() {captureLoop(*raw_stream);});
  }
}

CameraDriverNode::~CameraDriverNode()
{
  running_.store(false);
  for (auto & stream : streams_) {
    if (stream->thread.joinable()) {stream->thread.join();}
    stream->camera.close();
  }
}

void CameraDriverNode::captureLoop(Stream & stream)
{
  while (rclcpp::ok() && running_.load()) {
    DahengFrame frame;
    std::string error;
    if (!stream.camera.read(frame, error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s 相机取图失败：%s", stream.name.c_str(), error.c_str());
      continue;
    }
    sensor_msgs::msg::Image image;
    image.header.stamp = now();
    image.header.frame_id = stream.frame_id;
    image.height = frame.height; image.width = frame.width; image.encoding = "bgr8";
    image.step = frame.width * 3U; image.data = std::move(frame.bgr_data);
    auto info = stream.camera_info;
    info.header = image.header;
    stream.image_pub->publish(image);
    stream.camera_info_pub->publish(info);
  }
}

}  // camera_driver
