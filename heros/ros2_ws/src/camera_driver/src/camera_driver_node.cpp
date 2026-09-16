#include "camera_driver/camera_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <hero_msgs/msg/gimbal_state.hpp>

#include "camera_driver/camera_mode.hpp"
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
  for (auto directory = package_share;; directory = directory.parent_path()) {
    const auto candidate = directory / path;
    if (std::filesystem::exists(candidate)) {
      return candidate.string();
    }
    if (directory == directory.root_path()) {
      break;
    }
  }
  return (package_share / path).string();
}

}  // namespace

struct CameraDriverNode::Stream
{
  struct AimData
  {
    TimestampMode timestamp_mode{TimestampMode::kHost};
    double timestamp_offset{0.0};
    std::uint64_t device_timestamp_frequency_hz{0U};
    DeviceTimestampMapper timestamp_mapper;
    std::optional<std::int64_t> ros_to_steady_offset_ns;
    sensor_msgs::msg::CameraInfo camera_info;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
  };

  std::string name;
  CameraRole role{CameraRole::kAim8mm};
  std::string frame_id;
  CameraSource source{CameraSource::kCamera};
  DahengCamera camera;
  cv::VideoCapture video;
  bool video_loop{true};
  std::chrono::nanoseconds video_period{0};
  std::optional<rclcpp::Time> last_image_stamp;
  std::optional<AimData> aim;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
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
  declare_parameter<std::string>("gimbal_state_topic", "/hero/gimbal/state");
  for (const auto & name : {"aim8mm", "base"}) {
    declare_parameter<bool>(std::string(name) + ".enabled", true);
    declare_parameter<std::string>(std::string(name) + ".source", "camera");
    declare_parameter<std::string>(std::string(name) + ".serial_number", "");
    declare_parameter<std::string>(std::string(name) + ".video_path", "");
    declare_parameter<bool>(std::string(name) + ".video_loop", true);
    declare_parameter<double>(std::string(name) + ".video_rate_hz", 0.0);
    declare_parameter<std::string>(std::string(name) + ".frame_id",
      name == std::string("aim8mm") ? "camera_optical_frame" : "base_camera_optical_frame");
    declare_parameter<std::string>(std::string(name) + ".image_topic",
      std::string("/hero/camera/") + name + "/image_raw");
    declare_parameter<int>(std::string(name) + ".width", 1280);
    declare_parameter<int>(std::string(name) + ".height", 1024);
    declare_parameter<double>(std::string(name) + ".exposure_time_us",
      name == std::string("base") ? 12000.0 : 4000.0);
    declare_parameter<double>(std::string(name) + ".gain",
      name == std::string("base") ? 13.0 : 16.0);
  }

  declare_parameter<std::string>("aim8mm.timestamp_mode", "host");
  declare_parameter<double>("aim8mm.timestamp_offset", 0.0);
  declare_parameter<std::string>("aim8mm.camera_info_topic", "/hero/camera/aim8mm/camera_info");
  declare_parameter<double>("aim8mm.fx", 0.0);
  declare_parameter<double>("aim8mm.fy", 0.0);
  declare_parameter<double>("aim8mm.cx", 0.0);
  declare_parameter<double>("aim8mm.cy", 0.0);
  declare_parameter<double>("aim8mm.k1", 0.0);
  declare_parameter<double>("aim8mm.k2", 0.0);
  declare_parameter<double>("aim8mm.p1", 0.0);
  declare_parameter<double>("aim8mm.p2", 0.0);
  declare_parameter<double>("aim8mm.k3", 0.0);

  const auto configure_stream = [this](const char * name, CameraRole role) {
      if (!get_parameter(std::string(name) + ".enabled").as_bool()) {
        return;
      }
      auto stream = std::make_unique<Stream>();
      stream->name = name;
      stream->role = role;
      stream->frame_id = get_parameter(std::string(name) + ".frame_id").as_string();
      stream->source = parseCameraSource(get_parameter(std::string(name) + ".source").as_string());
      DahengCameraConfig config;
      config.serial_number = get_parameter(std::string(name) + ".serial_number").as_string();
      config.width = get_parameter(std::string(name) + ".width").as_int();
      config.height = get_parameter(std::string(name) + ".height").as_int();
      config.exposure_time_us = get_parameter(std::string(name) + ".exposure_time_us").as_double();
      config.gain = get_parameter(std::string(name) + ".gain").as_double();

      if (role == CameraRole::kAim8mm) {
        stream->aim.emplace();
        auto & aim = *stream->aim;
        aim.timestamp_mode = parseTimestampMode(get_parameter("aim8mm.timestamp_mode").as_string());
        aim.timestamp_offset = get_parameter("aim8mm.timestamp_offset").as_double();
        if (!std::isfinite(aim.timestamp_offset)) {
          throw std::runtime_error("aim8mm timestamp_offset 必须是有限数");
        }
      }

      if (stream->source == CameraSource::kCamera) {
        std::string error;
        if (!stream->camera.open(config, error)) {
          throw std::runtime_error(std::string(name) + " 相机打开失败：" + error);
        }
        if (stream->aim.has_value() &&
            stream->aim->timestamp_mode == TimestampMode::kDevice &&
            !stream->camera.timestampTickFrequencyHz(
              stream->aim->device_timestamp_frequency_hz, error)) {
          throw std::runtime_error("aim8mm 无法启用设备时间戳：" + error);
        }
      } else {
        if (stream->aim.has_value() &&
            stream->aim->timestamp_mode != TimestampMode::kHost) {
          throw std::runtime_error("视频输入只支持 aim8mm.timestamp_mode: host");
        }
        const auto video_path = resolveVideoPath(
          get_parameter(std::string(name) + ".video_path").as_string());
        if (!stream->video.open(video_path)) {
          throw std::runtime_error(std::string(name) + " 无法打开本地视频：" + video_path);
        }
        const auto video_width = static_cast<int>(stream->video.get(cv::CAP_PROP_FRAME_WIDTH));
        const auto video_height = static_cast<int>(stream->video.get(cv::CAP_PROP_FRAME_HEIGHT));
        if (video_width <= 0 || video_height <= 0) {
          throw std::runtime_error(std::string(name) + " 本地视频没有有效分辨率");
        }
        if (video_width != config.width || video_height != config.height) {
          RCLCPP_WARN(get_logger(), "%s 视频分辨率为 %dx%d，图像宽高将使用视频实际值",
            name, video_width, video_height);
        }
        config.width = video_width;
        config.height = video_height;
        stream->video_loop = get_parameter(std::string(name) + ".video_loop").as_bool();
        const auto configured_rate = get_parameter(std::string(name) + ".video_rate_hz").as_double();
        if (configured_rate < 0.0) {
          throw std::runtime_error(std::string(name) + " video_rate_hz 不能为负数");
        }
        const auto video_rate = stream->video.get(cv::CAP_PROP_FPS);
        const auto publish_rate = configured_rate > 0.0 ? configured_rate : video_rate;
        stream->video_period = publish_rate > 0.0 ?
          std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / publish_rate)) : std::chrono::milliseconds(33);
        if (publish_rate <= 0.0) {
          RCLCPP_WARN(get_logger(), "%s 视频未提供有效 FPS，按 30 Hz 回放", name);
        }
        RCLCPP_INFO(
          get_logger(), "%s 使用本地视频：%s，回放频率 %.2f Hz，%s循环", name,
          video_path.c_str(), 1.0 / std::chrono::duration<double>(stream->video_period).count(),
          stream->video_loop ? "启用" : "不启用");
      }

      const auto image_topic = get_parameter(std::string(name) + ".image_topic").as_string();
      if (image_topic.empty()) {
        throw std::runtime_error(std::string(name) + " image_topic 不能为空");
      }
      stream->image_pub = create_publisher<sensor_msgs::msg::Image>(image_topic, highRateQos());

      if (stream->aim.has_value()) {
        auto & aim = *stream->aim;
        const auto camera_info_topic = get_parameter("aim8mm.camera_info_topic").as_string();
        if (camera_info_topic.empty()) {
          throw std::runtime_error("aim8mm CameraInfo 话题不能为空");
        }
        aim.camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(
          camera_info_topic, highRateQos());
        aim.camera_info.width = static_cast<std::uint32_t>(config.width);
        aim.camera_info.height = static_cast<std::uint32_t>(config.height);
        aim.camera_info.distortion_model = "plumb_bob";
        const auto fx = get_parameter("aim8mm.fx").as_double();
        const auto fy = get_parameter("aim8mm.fy").as_double();
        const auto cx = get_parameter("aim8mm.cx").as_double();
        const auto cy = get_parameter("aim8mm.cy").as_double();
        aim.camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
        aim.camera_info.d = {
          get_parameter("aim8mm.k1").as_double(), get_parameter("aim8mm.k2").as_double(),
          get_parameter("aim8mm.p1").as_double(), get_parameter("aim8mm.p2").as_double(),
          get_parameter("aim8mm.k3").as_double()};
        aim.camera_info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
      }
      streams_.push_back(std::move(stream));
    };

  configure_stream("aim8mm", CameraRole::kAim8mm);
  configure_stream("base", CameraRole::kBase);
  if (streams_.empty()) {
    throw std::runtime_error("至少启用一路相机");
  }

  bool use_sim_time = false;
  get_parameter_or("use_sim_time", use_sim_time, false);
  for (const auto & stream : streams_) {
    if (use_sim_time && stream->aim.has_value() &&
        stream->aim->timestamp_mode == TimestampMode::kDevice) {
      throw std::runtime_error("设备时间戳模式不支持 use_sim_time");
    }
  }

  const auto gimbal_state_topic = get_parameter("gimbal_state_topic").as_string();
  if (gimbal_state_topic.empty()) {
    throw std::runtime_error("gimbal_state_topic 不能为空");
  }
  gimbal_state_sub_ = create_subscription<hero_msgs::msg::GimbalState>(
    gimbal_state_topic, highRateQos(),
    [this](const hero_msgs::msg::GimbalState::ConstSharedPtr state) {
      gimbal_mode_.store(state->mode);
    });

  running_.store(true);
  for (auto & stream : streams_) {
    stream->thread = std::thread([this, raw_stream = stream.get()]() { captureLoop(*raw_stream); });
  }
}

CameraDriverNode::~CameraDriverNode()
{
  running_.store(false);
  for (auto & stream : streams_) {
    if (stream->thread.joinable()) {
      stream->thread.join();
    }
    stream->camera.close();
  }
}

void CameraDriverNode::captureLoop(Stream & stream)
{
  auto next_video_frame_time = std::chrono::steady_clock::now();
  while (rclcpp::ok() && running_.load()) {
    const auto mode_before_capture = gimbal_mode_.load();
    const bool convert_to_bgr = isCameraRoleActive(stream.role, mode_before_capture);
    Frame frame;
    std::string error;
    if (!readFrame(stream, convert_to_bgr, frame, error)) {
      if (error.empty()) {
        RCLCPP_INFO(get_logger(), "%s 本地视频已播放结束", stream.name.c_str());
        return;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s 相机取图失败：%s", stream.name.c_str(), error.c_str());
      continue;
    }

    if (stream.source == CameraSource::kVideo) {
      next_video_frame_time += stream.video_period;
    }
    if (!convert_to_bgr || !shouldPublishFrame(
          stream.role, mode_before_capture, gimbal_mode_.load())) {
      if (stream.source == CameraSource::kVideo) {
        std::this_thread::sleep_until(next_video_frame_time);
      }
      continue;
    }

    const auto host_publish_stamp = now();
    rclcpp::Time host_receive_stamp{0, 0, RCL_ROS_TIME};
    const auto stamp = stream.aim.has_value() ?
      makeFrameStamp(stream, frame, host_publish_stamp, host_receive_stamp) :
      makeBaseFrameStamp(stream, frame, host_publish_stamp);
    if (!stamp.has_value()) {
      if (stream.source == CameraSource::kVideo) {
        std::this_thread::sleep_until(next_video_frame_time);
      }
      continue;
    }

    sensor_msgs::msg::Image image;
    image.header.stamp = *stamp;
    image.header.frame_id = stream.frame_id;
    image.height = frame.height;
    image.width = frame.width;
    image.encoding = "bgr8";
    image.step = frame.width * 3U;
    image.data = std::move(frame.bgr_data);
    stream.image_pub->publish(image);

    if (stream.aim.has_value()) {
      auto & aim = *stream.aim;
      auto info = aim.camera_info;
      info.header = image.header;
      aim.camera_info_pub->publish(info);
    }
    if (stream.source == CameraSource::kVideo) {
      std::this_thread::sleep_until(next_video_frame_time);
    }
  }
}

bool CameraDriverNode::readFrame(
  Stream & stream, bool convert_to_bgr, Frame & frame, std::string & error)
{
  if (stream.source == CameraSource::kVideo) {
    return readVideoFrame(stream, convert_to_bgr, frame, error);
  }

  DahengFrame daheng_frame;
  if (!stream.camera.read(daheng_frame, convert_to_bgr, error)) {
    return false;
  }
  if (!convert_to_bgr) {
    return true;
  }
  frame.width = daheng_frame.width;
  frame.height = daheng_frame.height;
  frame.device_timestamp = daheng_frame.device_timestamp;
  frame.host_receive_steady = daheng_frame.host_receive_steady;
  frame.bgr_data = std::move(daheng_frame.bgr_data);
  return true;
}

bool CameraDriverNode::readVideoFrame(
  Stream & stream, bool convert_to_bgr, Frame & frame, std::string & error)
{
  if (!convert_to_bgr) {
    if (stream.video.grab()) {
      return true;
    }
    if (!stream.video_loop) {
      error.clear();
      return false;
    }
    stream.video.set(cv::CAP_PROP_POS_FRAMES, 0.0);
    if (stream.video.grab()) {
      return true;
    }
    error = "本地视频无法读取有效图像帧";
    return false;
  }

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
  frame.host_receive_steady = std::chrono::steady_clock::now();

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
  frame.bgr_data.assign(
    bgr_image.data,
    bgr_image.data + static_cast<std::size_t>(frame.width) * frame.height * 3U);
  return true;
}

std::optional<rclcpp::Time> CameraDriverNode::makeBaseFrameStamp(
  Stream & stream, const Frame & frame, const rclcpp::Time & host_publish_stamp)
{
  const auto steady_now = std::chrono::steady_clock::now();
  const auto steady_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    steady_now.time_since_epoch()).count();
  const auto receive_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    frame.host_receive_steady.time_since_epoch()).count();
  const auto stamp = rclcpp::Time(
    receive_steady_ns + host_publish_stamp.nanoseconds() - steady_now_ns, RCL_ROS_TIME);
  if (stream.last_image_stamp.has_value() && stamp <= *stream.last_image_stamp) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s 主机时间戳未递增，丢弃当前图像", stream.name.c_str());
    return std::nullopt;
  }
  stream.last_image_stamp = stamp;
  return stamp;
}

std::optional<rclcpp::Time> CameraDriverNode::makeFrameStamp(
  Stream & stream, const Frame & frame, const rclcpp::Time & host_publish_stamp,
  rclcpp::Time & host_receive_stamp)
{
  auto & aim = *stream.aim;
  const auto steady_now = std::chrono::steady_clock::now();
  const auto steady_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    steady_now.time_since_epoch()).count();
  const auto ros_to_steady_ns = host_publish_stamp.nanoseconds() - steady_now_ns;
  const auto receive_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    frame.host_receive_steady.time_since_epoch()).count();
  host_receive_stamp = rclcpp::Time(receive_steady_ns + ros_to_steady_ns, RCL_ROS_TIME);
  if (aim.timestamp_mode == TimestampMode::kHost) {
    const auto stamp = host_receive_stamp +
      rclcpp::Duration::from_seconds(aim.timestamp_offset);
    if (stream.last_image_stamp.has_value() && stamp <= *stream.last_image_stamp) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s 主机时间戳未递增，丢弃当前图像", stream.name.c_str());
      return std::nullopt;
    }
    stream.last_image_stamp = stamp;
    return stamp;
  }

  if (aim.ros_to_steady_offset_ns.has_value() && hasRosSteadyClockJump(
      *aim.ros_to_steady_offset_ns, ros_to_steady_ns,
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(100)).count())) {
    aim.timestamp_mapper.reset();
    aim.ros_to_steady_offset_ns.reset();
    stream.last_image_stamp.reset();
    RCLCPP_WARN(get_logger(), "%s 检测到 ROS 时间跳变，已重置设备时间映射并丢弃当前图像",
      stream.name.c_str());
    return std::nullopt;
  }
  aim.ros_to_steady_offset_ns = ros_to_steady_ns;
  TimestampMapping mapping;
  try {
    mapping = aim.timestamp_mapper.update(
      frame.device_timestamp, aim.device_timestamp_frequency_hz, receive_steady_ns);
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s 设备时间戳映射失败：%s",
      stream.name.c_str(), error.what());
    return std::nullopt;
  }
  if (mapping.update == TimestampUpdate::kReset) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s 设备时间戳未递增或频率变化，已重置映射并丢弃当前图像",
      stream.name.c_str());
    return std::nullopt;
  }
  const auto mapped_stamp = rclcpp::Time(
    mapping.mapped_host_steady_ns + ros_to_steady_ns, RCL_ROS_TIME);
  const auto stamp = std::min(mapped_stamp, host_receive_stamp) +
    rclcpp::Duration::from_seconds(aim.timestamp_offset);
  if (stream.last_image_stamp.has_value() && stamp <= *stream.last_image_stamp) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s 设备映射时间戳未递增，丢弃当前图像", stream.name.c_str());
    return std::nullopt;
  }
  stream.last_image_stamp = stamp;
  return stamp;
}

}  // namespace camera_driver
