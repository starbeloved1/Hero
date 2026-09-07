#include "camera_driver/camera_driver_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <hero_msgs/msg/camera_timing.hpp>

#include "camera_driver/camera_source.hpp"
#include "camera_driver/daheng_camera.hpp"
#include "camera_driver/timestamp_mapper.hpp"

namespace camera_driver
{

namespace
{

rclcpp::QoS highRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

std::string resolveVideoPath(const std::string & configured_path)
{
  if (configured_path.empty()) {
    throw std::invalid_argument("本地视频路径不能为空");
  }
  const std::filesystem::path path(configured_path);
  if (path.is_absolute()) {
    return path.string();
  }

  const std::filesystem::path package_share(
    ament_index_cpp::get_package_share_directory("camera_driver"));
  for (auto directory = package_share; !directory.empty(); directory = directory.parent_path()) {
    const auto candidate = directory / path;
    if (std::filesystem::exists(candidate)) {
      return candidate.string();
    }
  }
  return (package_share / path).string();
}

}  // namespace

struct CameraDriverNode::Stream
{
  std::string name;
  std::string frame_id;
  CameraSource source{CameraSource::kDaheng};
  TimestampMode timestamp_mode{TimestampMode::kHost};
  DahengCamera camera;
  cv::VideoCapture video;
  bool video_loop{true};
  std::chrono::nanoseconds video_period{0};
  double timestamp_offset_sec{0.0};
  std::uint64_t device_timestamp_frequency_hz{0U};
  DeviceTimestampMapper timestamp_mapper;
  std::optional<std::int64_t> ros_to_steady_offset_ns;
  std::optional<rclcpp::Time> last_image_stamp;
  sensor_msgs::msg::CameraInfo camera_info;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
  rclcpp::Publisher<hero_msgs::msg::CameraTiming>::SharedPtr timing_pub;
  std::thread thread;
};

struct CameraDriverNode::Frame
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint64_t device_timestamp{0U};
  std::chrono::steady_clock::time_point host_receive_steady{};
  std::vector<std::uint8_t> bgr_data;
};

CameraDriverNode::CameraDriverNode()
: Node("camera_driver_node")
{
  declare_parameter<bool>("aim8mm.enabled", true);
  declare_parameter<bool>("base.enabled", true);
  for (const auto & name : {"aim8mm", "base"}) {
    declare_parameter<std::string>(std::string(name) + ".source", "daheng");
    declare_parameter<std::string>(std::string(name) + ".serial_number", "");
    declare_parameter<std::string>(std::string(name) + ".video_path", "");
    declare_parameter<bool>(std::string(name) + ".video_loop", true);
    declare_parameter<double>(std::string(name) + ".video_rate_hz", 0.0);
    declare_parameter<std::string>(std::string(name) + ".timestamp_mode", "host");
    declare_parameter<double>(std::string(name) + ".timestamp_offset_sec", 0.0);
    declare_parameter<std::string>(
      std::string(name) + ".timing_topic", std::string("/hero/camera/") + name + "/timing");
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
    stream->source = parseCameraSource(get_parameter(std::string(name) + ".source").as_string());
    stream->timestamp_mode = parseTimestampMode(
      get_parameter(std::string(name) + ".timestamp_mode").as_string());
    stream->timestamp_offset_sec =
      get_parameter(std::string(name) + ".timestamp_offset_sec").as_double();
    if (!std::isfinite(stream->timestamp_offset_sec)) {
      throw std::runtime_error(name + std::string(" timestamp_offset_sec 必须是有限数"));
    }
    DahengCameraConfig config;
    config.serial_number = get_parameter(std::string(name) + ".serial_number").as_string();
    config.width = get_parameter(std::string(name) + ".width").as_int();
    config.height = get_parameter(std::string(name) + ".height").as_int();
    config.exposure_time_us = get_parameter(std::string(name) + ".exposure_time_us").as_double();
    config.gain = get_parameter(std::string(name) + ".gain").as_double();
    if (stream->source == CameraSource::kDaheng) {
      std::string error;
      if (!stream->camera.open(config, error)) {
        throw std::runtime_error(name + std::string(" 相机打开失败：") + error);
      }
      if (stream->timestamp_mode == TimestampMode::kDevice &&
          !stream->camera.timestampTickFrequencyHz(
            stream->device_timestamp_frequency_hz, error)) {
        throw std::runtime_error(name + std::string(" 无法启用设备时间戳：") + error);
      }
    } else {
      if (stream->timestamp_mode != TimestampMode::kHost) {
        throw std::runtime_error(name + std::string(" 视频输入只支持 timestamp_mode: host"));
      }
      const auto video_path = resolveVideoPath(
        get_parameter(std::string(name) + ".video_path").as_string());
      if (!stream->video.open(video_path)) {
        throw std::runtime_error(name + std::string(" 无法打开本地视频：") + video_path);
      }
      const auto video_width = static_cast<int>(stream->video.get(cv::CAP_PROP_FRAME_WIDTH));
      const auto video_height = static_cast<int>(stream->video.get(cv::CAP_PROP_FRAME_HEIGHT));
      if (video_width <= 0 || video_height <= 0) {
        throw std::runtime_error(name + std::string(" 本地视频没有有效分辨率"));
      }
      if (video_width != config.width || video_height != config.height) {
        RCLCPP_WARN(
          get_logger(), "%s 视频分辨率为 %dx%d，CameraInfo 宽高将使用视频实际值；"
          "请确认 YAML 内参对应此分辨率", name, video_width, video_height);
      }
      config.width = video_width;
      config.height = video_height;
      stream->video_loop = get_parameter(std::string(name) + ".video_loop").as_bool();
      const auto configured_rate = get_parameter(std::string(name) + ".video_rate_hz").as_double();
      if (configured_rate < 0.0) {
        throw std::runtime_error(name + std::string(" video_rate_hz 不能为负数"));
      }
      const auto video_rate = stream->video.get(cv::CAP_PROP_FPS);
      const auto publish_rate = configured_rate > 0.0 ? configured_rate : video_rate;
      if (publish_rate > 0.0) {
        stream->video_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / publish_rate));
      } else {
        stream->video_period = std::chrono::milliseconds(33);
        RCLCPP_WARN(
          get_logger(), "%s 视频未提供有效 FPS，按 30 Hz 回放", name);
      }
      RCLCPP_INFO(
        get_logger(), "%s 使用本地视频：%s，回放频率 %.2f Hz，%s循环", name, video_path.c_str(),
        1.0 / std::chrono::duration<double>(stream->video_period).count(),
        stream->video_loop ? "启用" : "不启用");
    }
    const auto image_topic = get_parameter(std::string(name) + ".image_topic").as_string();
    const auto camera_info_topic = get_parameter(std::string(name) + ".camera_info_topic").as_string();
    if (image_topic.empty() || camera_info_topic.empty()) {
      throw std::runtime_error(name + std::string(" 相机话题不能为空"));
    }
    stream->image_pub = create_publisher<sensor_msgs::msg::Image>(image_topic, highRateQos());
    stream->camera_info_pub =
      create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, highRateQos());
    const auto timing_topic = get_parameter(std::string(name) + ".timing_topic").as_string();
    if (timing_topic.empty()) {
      throw std::runtime_error(name + std::string(" timing_topic 不能为空"));
    }
    stream->timing_pub = create_publisher<hero_msgs::msg::CameraTiming>(timing_topic, highRateQos());
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
  bool use_sim_time = false;
  get_parameter_or("use_sim_time", use_sim_time, false);
  if (use_sim_time) {
    for (const auto & stream : streams_) {
      if (stream->timestamp_mode == TimestampMode::kDevice) {
        throw std::runtime_error("设备时间戳模式不支持 use_sim_time");
      }
    }
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
  // 本地视频没有硬件帧到达事件。以绝对时间轴回放，避免把解码和 ROS
  // 发布耗时额外累加到每一帧的 sleep 中而让整段视频逐渐变慢
  auto next_video_frame_time = std::chrono::steady_clock::now();
  while (rclcpp::ok() && running_.load()) {
    Frame frame;
    std::string error;
    if (!readFrame(stream, frame, error)) {
      if (error.empty()) {
        RCLCPP_INFO(get_logger(), "%s 本地视频已播放结束", stream.name.c_str());
        return;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s 相机取图失败：%s", stream.name.c_str(), error.c_str());
      continue;
    }
    const auto host_publish_stamp = now();
    rclcpp::Time host_receive_stamp{0, 0, RCL_ROS_TIME};
    double mapping_offset_sec = 0.0;
    const auto stamp = makeFrameStamp(
      stream, frame, host_publish_stamp, host_receive_stamp, mapping_offset_sec);
    if (!stamp.has_value()) {
      continue;
    }
    sensor_msgs::msg::Image image;
    image.header.stamp = *stamp;
    image.header.frame_id = stream.frame_id;
    image.height = frame.height; image.width = frame.width; image.encoding = "bgr8";
    image.step = frame.width * 3U; image.data = std::move(frame.bgr_data);
    auto info = stream.camera_info;
    info.header = image.header;
    stream.image_pub->publish(image);
    stream.camera_info_pub->publish(info);
    if (stream.timing_pub->get_subscription_count() > 0U) {
      hero_msgs::msg::CameraTiming timing;
      timing.header = image.header;
      timing.device_timestamp =
        stream.source == CameraSource::kDaheng ? frame.device_timestamp : 0U;
      timing.device_timestamp_frequency_hz = stream.device_timestamp_frequency_hz;
      timing.host_receive_stamp = host_receive_stamp;
      timing.host_publish_stamp = host_publish_stamp;
      timing.timestamp_mode = timestampModeName(stream.timestamp_mode);
      timing.device_to_host_offset_sec = mapping_offset_sec;
      timing.reset_count = stream.timestamp_mapper.resetCount();
      stream.timing_pub->publish(timing);
    }
    if (stream.source == CameraSource::kVideo) {
      next_video_frame_time += stream.video_period;
      std::this_thread::sleep_until(next_video_frame_time);
    }
  }
}

bool CameraDriverNode::readFrame(Stream & stream, Frame & frame, std::string & error)
{
  if (stream.source == CameraSource::kVideo) {
    return readVideoFrame(stream, frame, error);
  }

  DahengFrame daheng_frame;
  if (!stream.camera.read(daheng_frame, error)) {
    return false;
  }
  frame.width = daheng_frame.width;
  frame.height = daheng_frame.height;
  frame.device_timestamp = daheng_frame.device_timestamp;
  frame.host_receive_steady = daheng_frame.host_receive_steady;
  frame.bgr_data = std::move(daheng_frame.bgr_data);
  return true;
}

bool CameraDriverNode::readVideoFrame(Stream & stream, Frame & frame, std::string & error)
{
  cv::Mat source_image;
  if (!stream.video.read(source_image) || source_image.empty()) {
    if (!stream.video_loop) {
      error.clear();
      return false;
    }
    stream.video.set(cv::CAP_PROP_POS_FRAMES, 0.0);
    if (!stream.video.read(source_image) || source_image.empty()) {
      error = "本地视频无法读取有效图像帧";
      return false;
    }
  }

  cv::Mat bgr_image;
  if (source_image.channels() == 3) {
    bgr_image = source_image;
  } else if (source_image.channels() == 1) {
    cv::cvtColor(source_image, bgr_image, cv::COLOR_GRAY2BGR);
  } else if (source_image.channels() == 4) {
    cv::cvtColor(source_image, bgr_image, cv::COLOR_BGRA2BGR);
  } else {
    error = "本地视频图像通道数不受支持";
    return false;
  }
  if (bgr_image.depth() != CV_8U) {
    error = "本地视频图像不是 8 位格式";
    return false;
  }
  if (!bgr_image.isContinuous()) {
    bgr_image = bgr_image.clone();
  }
  frame.width = static_cast<std::uint32_t>(bgr_image.cols);
  frame.height = static_cast<std::uint32_t>(bgr_image.rows);
  frame.host_receive_steady = std::chrono::steady_clock::now();
  frame.bgr_data.assign(
    bgr_image.data,
    bgr_image.data + static_cast<std::size_t>(frame.width) * frame.height * 3U);
  return true;
}

std::optional<rclcpp::Time> CameraDriverNode::makeFrameStamp(
  Stream & stream, const Frame & frame, const rclcpp::Time & host_publish_stamp,
  rclcpp::Time & host_receive_stamp, double & mapping_offset_sec)
{
  mapping_offset_sec = 0.0;
  const auto steady_now = std::chrono::steady_clock::now();
  const auto steady_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    steady_now.time_since_epoch()).count();
  const auto ros_now_ns = host_publish_stamp.nanoseconds();
  const auto ros_to_steady_ns = ros_now_ns - steady_now_ns;
  const auto receive_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    frame.host_receive_steady.time_since_epoch()).count();
  host_receive_stamp = rclcpp::Time(receive_steady_ns + ros_to_steady_ns, RCL_ROS_TIME);
  if (stream.timestamp_mode == TimestampMode::kHost) {
    const auto stamp = host_publish_stamp +
      rclcpp::Duration::from_seconds(stream.timestamp_offset_sec);
    if (stream.last_image_stamp.has_value() && stamp <= *stream.last_image_stamp) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s host 时间戳未递增，丢弃当前图像", stream.name.c_str());
      return std::nullopt;
    }
    stream.last_image_stamp = stamp;
    return stamp;
  }

  if (stream.ros_to_steady_offset_ns.has_value() &&
      hasRosSteadyClockJump(
        *stream.ros_to_steady_offset_ns, ros_to_steady_ns,
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::milliseconds(100)).count())) {
    stream.timestamp_mapper.reset();
    stream.ros_to_steady_offset_ns.reset();
    stream.last_image_stamp.reset();
    RCLCPP_WARN(get_logger(), "%s 检测到 ROS 时间跳变，已重置设备时间映射并丢弃当前图像",
                stream.name.c_str());
    return std::nullopt;
  }
  stream.ros_to_steady_offset_ns = ros_to_steady_ns;
  TimestampMapping mapping;
  try {
    mapping = stream.timestamp_mapper.update(
      frame.device_timestamp, stream.device_timestamp_frequency_hz, receive_steady_ns);
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "%s 设备时间戳映射失败：%s", stream.name.c_str(), error.what());
    return std::nullopt;
  }
  if (mapping.update == TimestampUpdate::kReset) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "%s 设备时间戳未递增或频率变化，已重置映射并丢弃当前图像",
                         stream.name.c_str());
    return std::nullopt;
  }
  mapping_offset_sec = mapping.residual_offset_sec;
  const auto mapped_stamp = rclcpp::Time(
    mapping.mapped_host_steady_ns + ros_to_steady_ns, RCL_ROS_TIME);
  // 未加人工补偿时，设备映射不能晚于该帧已经到达主机的时刻。
  const auto base_stamp = std::min(mapped_stamp, host_receive_stamp);
  const auto stamp = base_stamp +
    rclcpp::Duration::from_seconds(stream.timestamp_offset_sec);
  if (stream.last_image_stamp.has_value() && stamp <= *stream.last_image_stamp) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "%s 设备映射时间戳未递增，丢弃当前图像", stream.name.c_str());
    return std::nullopt;
  }
  stream.last_image_stamp = stamp;
  return stamp;
}

}  // camera_driver
