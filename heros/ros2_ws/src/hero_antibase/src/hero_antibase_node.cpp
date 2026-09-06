#include "hero_antibase/hero_antibase_node.hpp"

#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.h>

namespace hero_antibase {

namespace {

rclcpp::QoS highRateQos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

rclcpp::QoS packetQos(int depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
}

} // namespace

HeroAntiBaseNode::HeroAntiBaseNode() : Node("hero_antibase_node") {
  declare_parameter<std::string>("image_topic",
                                 "/hero/camera/selected/image_raw");
  declare_parameter<std::string>("gimbal_state_topic", "/hero/gimbal/state");
  declare_parameter<std::string>("packet_topic", "/hero/antibase/packets");
  declare_parameter<std::string>("tx_status_topic",
                                 "/hero/antibase/tx_status");
  declare_parameter<std::string>("debug_topic", "/hero/antibase/debug");
  declare_parameter<bool>("visualization_enabled", false);
  declare_parameter<std::string>("visualization_topic",
                                 "/hero/antibase/visualization");
  declare_parameter<double>("visualization_rate_hz", 10.0);
  declare_parameter<int>("packet_queue_depth", 64);

  AntiBaseConfig config;
  declare_parameter<int>("output_fps", config.output_fps);
  declare_parameter<int>("crop_size", config.crop_size);
  declare_parameter<int>("crop_offset_x", config.crop_offset_x);
  declare_parameter<int>("output_size", config.output_size);
  declare_parameter<bool>("static_simplify", config.static_simplify);
  declare_parameter<int>("motion_threshold", config.motion_threshold);
  declare_parameter<int>("motion_erode_px", config.motion_erode_px);
  declare_parameter<int>("motion_dilate_px", config.motion_dilate_px);
  declare_parameter<int>("motion_trail_frames", config.motion_trail_frames);
  declare_parameter<double>("trail_brightness_gain",
                            config.trail_brightness_gain);
  declare_parameter<double>("trail_disable_motion_ratio",
                            config.trail_disable_motion_ratio);
  declare_parameter<double>("trail_resume_motion_ratio",
                            config.trail_resume_motion_ratio);
  declare_parameter<double>("motion_ratio_ema_alpha",
                            config.motion_ratio_ema_alpha);
  declare_parameter<double>("bg_update_alpha", config.bg_update_alpha);
  declare_parameter<double>("bg_blur_sigma", config.bg_blur_sigma);
  declare_parameter<int>("center_clear_size", config.center_clear_size);
  declare_parameter<bool>("force_monochrome", config.force_monochrome);
  declare_parameter<int>("target_bitrate_kbps", config.target_bitrate_kbps);
  declare_parameter<double>("max_tx_delay_sec", config.max_tx_delay_sec);
  declare_parameter<double>("min_packet_gap_ms", config.min_packet_gap_ms);
  declare_parameter<bool>("adaptive_bitrate_enabled",
                          config.adaptive_bitrate_enabled);
  declare_parameter<int>("adaptive_bitrate_min_kbps",
                         config.adaptive_bitrate_min_kbps);
  declare_parameter<int>("adaptive_bitrate_max_kbps",
                         config.adaptive_bitrate_max_kbps);
  declare_parameter<int>("adaptive_bitrate_step_up_kbps",
                         config.adaptive_bitrate_step_up_kbps);
  declare_parameter<int>("adaptive_bitrate_step_down_kbps",
                         config.adaptive_bitrate_step_down_kbps);
  declare_parameter<double>("adaptive_bitrate_control_interval_sec",
                            config.adaptive_bitrate_control_interval_sec);
  declare_parameter<int>("h264_speed_preset", config.h264_speed_preset);
  declare_parameter<int>("h264_tune", config.h264_tune);
  declare_parameter<int>("h264_key_int_max", config.h264_key_int_max);
  declare_parameter<int>("h264_bframes", config.h264_bframes);
  declare_parameter<int>("h264_rc_lookahead", config.h264_rc_lookahead);
  declare_parameter<int>("h264_sync_lookahead", config.h264_sync_lookahead);
  declare_parameter<bool>("h264_sliced_threads", config.h264_sliced_threads);
  declare_parameter<int>("h264_ref", config.h264_ref);
  declare_parameter<int>("h264_vbv_buf_capacity", config.h264_vbv_buf_capacity);
  declare_parameter<std::string>("h264_option_string",
                                 config.h264_option_string);

  config.output_fps = get_parameter("output_fps").as_int();
  config.crop_size = get_parameter("crop_size").as_int();
  config.crop_offset_x = get_parameter("crop_offset_x").as_int();
  config.output_size = get_parameter("output_size").as_int();
  config.static_simplify = get_parameter("static_simplify").as_bool();
  config.motion_threshold = get_parameter("motion_threshold").as_int();
  config.motion_erode_px = get_parameter("motion_erode_px").as_int();
  config.motion_dilate_px = get_parameter("motion_dilate_px").as_int();
  config.motion_trail_frames = get_parameter("motion_trail_frames").as_int();
  config.trail_brightness_gain =
      get_parameter("trail_brightness_gain").as_double();
  config.trail_disable_motion_ratio =
      get_parameter("trail_disable_motion_ratio").as_double();
  config.trail_resume_motion_ratio =
      get_parameter("trail_resume_motion_ratio").as_double();
  config.motion_ratio_ema_alpha =
      get_parameter("motion_ratio_ema_alpha").as_double();
  config.bg_update_alpha = get_parameter("bg_update_alpha").as_double();
  config.bg_blur_sigma = get_parameter("bg_blur_sigma").as_double();
  config.center_clear_size = get_parameter("center_clear_size").as_int();
  config.force_monochrome = get_parameter("force_monochrome").as_bool();
  config.target_bitrate_kbps = get_parameter("target_bitrate_kbps").as_int();
  config.max_tx_delay_sec = get_parameter("max_tx_delay_sec").as_double();
  config.min_packet_gap_ms = get_parameter("min_packet_gap_ms").as_double();
  config.adaptive_bitrate_enabled =
      get_parameter("adaptive_bitrate_enabled").as_bool();
  config.adaptive_bitrate_min_kbps =
      get_parameter("adaptive_bitrate_min_kbps").as_int();
  config.adaptive_bitrate_max_kbps =
      get_parameter("adaptive_bitrate_max_kbps").as_int();
  config.adaptive_bitrate_step_up_kbps =
      get_parameter("adaptive_bitrate_step_up_kbps").as_int();
  config.adaptive_bitrate_step_down_kbps =
      get_parameter("adaptive_bitrate_step_down_kbps").as_int();
  config.adaptive_bitrate_control_interval_sec =
      get_parameter("adaptive_bitrate_control_interval_sec").as_double();
  config.h264_speed_preset = get_parameter("h264_speed_preset").as_int();
  config.h264_tune = get_parameter("h264_tune").as_int();
  config.h264_key_int_max = get_parameter("h264_key_int_max").as_int();
  config.h264_bframes = get_parameter("h264_bframes").as_int();
  config.h264_rc_lookahead = get_parameter("h264_rc_lookahead").as_int();
  config.h264_sync_lookahead = get_parameter("h264_sync_lookahead").as_int();
  config.h264_sliced_threads = get_parameter("h264_sliced_threads").as_bool();
  config.h264_ref = get_parameter("h264_ref").as_int();
  config.h264_vbv_buf_capacity =
      get_parameter("h264_vbv_buf_capacity").as_int();
  config.h264_option_string = get_parameter("h264_option_string").as_string();
  processor_ = std::make_unique<AntiBaseProcessor>(std::move(config));

  const auto depth = get_parameter("packet_queue_depth").as_int();
  if (depth <= 0) {
    throw std::invalid_argument("packet_queue_depth 必须为正");
  }
  packet_pub_ = create_publisher<hero_msgs::msg::AntiBasePacket>(
      get_parameter("packet_topic").as_string(), packetQos(depth));
  debug_pub_ = create_publisher<hero_msgs::msg::AntiBaseDebug>(
      get_parameter("debug_topic").as_string(), highRateQos());
  if (get_parameter("visualization_enabled").as_bool()) {
    const auto rate = get_parameter("visualization_rate_hz").as_double();
    if (rate < 0.0) {
      throw std::invalid_argument("visualization_rate_hz 不能为负数");
    }
    // 0 表示不额外抽帧，逐个已处理输入发布可视化；正数才限频
    if (rate > 0.0) {
      visualization_period_ =
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(1.0 / rate));
    }
    visualization_pub_ = create_publisher<sensor_msgs::msg::Image>(
        get_parameter("visualization_topic").as_string(), highRateQos());
  }
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      get_parameter("image_topic").as_string(), highRateQos(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr image) {
        receiveImage(image);
      });
  gimbal_state_sub_ = create_subscription<hero_msgs::msg::GimbalState>(
      get_parameter("gimbal_state_topic").as_string(), highRateQos(),
      [this](const hero_msgs::msg::GimbalState::ConstSharedPtr state) {
        receiveGimbalState(state);
      });
  tx_status_sub_ = create_subscription<hero_msgs::msg::AntiBaseTxStatus>(
      get_parameter("tx_status_topic").as_string(), packetQos(1),
      [this](const hero_msgs::msg::AntiBaseTxStatus::ConstSharedPtr status) {
        receiveTxStatus(status);
      });
  running_.store(true);
  worker_ = std::thread([this]() { processLoop(); });
  RCLCPP_INFO(get_logger(), "反基地节点已启动：仅在 mode4 编码 %s 并发布 %s",
              get_parameter("image_topic").as_string().c_str(),
              get_parameter("packet_topic").as_string().c_str());
}

HeroAntiBaseNode::~HeroAntiBaseNode() {
  running_.store(false);
  image_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void HeroAntiBaseNode::receiveImage(
    const sensor_msgs::msg::Image::ConstSharedPtr &image) {
  if (mode_.load() != hero_msgs::msg::GimbalState::MODE_ANTI_BASE) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image_ = image;
  }
  image_cv_.notify_one();
}

void HeroAntiBaseNode::receiveGimbalState(
    const hero_msgs::msg::GimbalState::ConstSharedPtr &state) {
  const auto previous = mode_.exchange(state->mode);
  if (previous != state->mode) {
    reset_requested_.store(true);
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image_.reset();
  }
}

void HeroAntiBaseNode::receiveTxStatus(
    const hero_msgs::msg::AntiBaseTxStatus::ConstSharedPtr &status) {
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  feedback_.sent_packets = status->sent_packets;
  feedback_.pending_packets = status->pending_packets;
  feedback_.oldest_pending_ms = status->oldest_pending_ms;
}

void HeroAntiBaseNode::processLoop() {
  while (rclcpp::ok() && running_.load()) {
    sensor_msgs::msg::Image::ConstSharedPtr image;
    {
      std::unique_lock<std::mutex> lock(image_mutex_);
      image_cv_.wait(lock, [this]() {
        return !running_.load() || latest_image_ != nullptr;
      });
      if (!running_.load()) {
        return;
      }
      image = std::move(latest_image_);
      latest_image_.reset();
    }
    if (mode_.load() != hero_msgs::msg::GimbalState::MODE_ANTI_BASE) {
      continue;
    }
    try {
      const auto cv_image = cv_bridge::toCvShare(image, "bgr8");
      TxFeedback feedback;
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        feedback = feedback_;
      }
      ProcessorResult result;
      {
        std::lock_guard<std::mutex> lock(processor_mutex_);
        if (reset_requested_.exchange(false)) {
          processor_->reset();
        }
        result = processor_->process(cv_image->image, feedback);
      }
      if (mode_.load() == hero_msgs::msg::GimbalState::MODE_ANTI_BASE) {
        publishResult(image, result);
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "反基地处理失败：%s", error.what());
    }
  }
}

void HeroAntiBaseNode::publishResult(
    const sensor_msgs::msg::Image::ConstSharedPtr &image,
    const ProcessorResult &result) {
  for (const auto &packet : result.packets) {
    hero_msgs::msg::AntiBasePacket message;
    message.header = image->header;
    message.sequence_id = packet.sequence_id;
    message.data = packet.data;
    packet_pub_->publish(message);
  }
  hero_msgs::msg::AntiBaseDebug debug;
  debug.header = image->header;
  debug.encoded_bytes = result.encoded_bytes;
  debug.published_packets = static_cast<uint32_t>(result.packets.size());
  debug.motion_ratio = result.motion_ratio;
  debug.suppress_trail = result.suppress_trail;
  debug.bitrate_kbps = result.bitrate_kbps;
  debug.raw_backlog_bytes = result.raw_backlog_bytes;
  debug.dropped_bytes = result.dropped_bytes;
  debug_pub_->publish(debug);
  if (shouldPublishVisualization() && !result.visualization.empty()) {
    auto message =
        cv_bridge::CvImage(image->header, "bgr8", result.visualization)
            .toImageMsg();
    visualization_pub_->publish(*message);
  }
}

bool HeroAntiBaseNode::shouldPublishVisualization() {
  if (!visualization_pub_ ||
      visualization_pub_->get_subscription_count() == 0U) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (visualization_period_.count() > 0 &&
      last_visualization_time_.time_since_epoch().count() != 0 &&
      now - last_visualization_time_ < visualization_period_) {
    return false;
  }
  last_visualization_time_ = now;
  return true;
}

} // namespace hero_antibase
