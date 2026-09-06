#include "hero_antibase/antibase_processor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <opencv2/imgproc.hpp>

namespace hero_antibase {

namespace {

constexpr std::size_t kPacketBytes = 300U;
constexpr std::size_t kPacketPayloadBytes = 292U;

int64_t steadyNowNs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

} // namespace

struct AntiBaseProcessor::GstState {
  GstElement *pipeline{nullptr};
  GstElement *appsrc{nullptr};
  GstElement *appsink{nullptr};
  GstElement *encoder{nullptr};
};

AntiBaseProcessor::AntiBaseProcessor(AntiBaseConfig config)
    : config_(std::move(config)),
      current_bitrate_kbps_(config_.target_bitrate_kbps), gst_(new GstState()) {
  if (config_.output_fps <= 0 || config_.output_size <= 0 ||
      config_.crop_size <= 0 || config_.min_packet_gap_ms <= 20.0) {
    throw std::invalid_argument(
        "反基地输出帧率、尺寸必须为正，完整包间隔必须大于 20 ms");
  }
  const auto bytes_per_second = static_cast<double>(kPacketBytes) /
                                config_.min_packet_gap_ms * 1000.0;
  max_backlog_bytes_ = static_cast<std::size_t>(std::max(
      1.0, bytes_per_second * std::max(0.01, config_.max_tx_delay_sec)));
  initializeEncoder();
}

AntiBaseProcessor::~AntiBaseProcessor() {
  shutdownEncoder();
  delete gst_;
}

void AntiBaseProcessor::reset() {
  background_gray_f32_.release();
  erode_kernel_.release();
  dilate_kernel_.release();
  trail_masks_.clear();
  trail_frames_.clear();
  smoothed_motion_ratio_ = 0.0;
  motion_initialized_ = false;
  global_motion_active_ = false;
  stream_buffer_.clear();
  dropped_bytes_ = 0U;
  sequence_id_ = 0U;
  next_packet_admit_ns_ = 0;
  last_encode_ns_ = 0;
  last_control_ns_ = 0;
  last_feedback_ns_ = 0;
  last_sent_packets_ = 0U;
  encoded_bytes_since_control_ = 0U;
  last_control_dropped_bytes_ = 0U;
  shutdownEncoder();
  initializeEncoder();
}

ProcessorResult AntiBaseProcessor::process(const cv::Mat &image,
                                           const TxFeedback &feedback) {
  ProcessorResult result;
  result.bitrate_kbps = current_bitrate_kbps_;
  if (image.empty()) {
    return result;
  }
  updateBitrate(feedback);
  const auto now_ns = steadyNowNs();
  const auto interval_ns = 1000000000LL / config_.output_fps;
  if (last_encode_ns_ != 0 &&
      now_ns - last_encode_ns_ < interval_ns * 19 / 20) {
    result.packets = popPackets();
    result.raw_backlog_bytes = static_cast<uint32_t>(stream_buffer_.size());
    result.dropped_bytes = dropped_bytes_;
    return result;
  }
  last_encode_ns_ = now_ns;
  const auto prepared = preprocess(image, result);
  result.visualization = prepared;
  const auto encoded = encode(prepared);
  result.encoded_bytes = static_cast<uint32_t>(encoded.size());
  encoded_bytes_since_control_ += encoded.size();
  stream_buffer_.insert(stream_buffer_.end(), encoded.begin(), encoded.end());
  result.packets = popPackets();
  result.bitrate_kbps = current_bitrate_kbps_;
  result.raw_backlog_bytes = static_cast<uint32_t>(stream_buffer_.size());
  result.dropped_bytes = dropped_bytes_;
  return result;
}

cv::Mat AntiBaseProcessor::preprocess(const cv::Mat &image,
                                      ProcessorResult &result) {
  const int side = std::min({config_.crop_size, image.cols, image.rows});
  const int centered_x = std::max(0, (image.cols - side) / 2);
  const int x = std::clamp(centered_x + config_.crop_offset_x, 0,
                           std::max(0, image.cols - side));
  const int y = std::max(0, (image.rows - side) / 2);
  cv::Mat resized;
  cv::resize(image(cv::Rect(x, y, side, side)), resized,
             cv::Size(config_.output_size, config_.output_size), 0.0, 0.0,
             cv::INTER_LINEAR);
  cv::Mat working = resized;
  if (config_.force_monochrome) {
    cv::Mat gray;
    cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray, working, cv::COLOR_GRAY2BGR);
  }
  cv::Mat gray;
  cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
  if (background_gray_f32_.empty()) {
    gray.convertTo(background_gray_f32_, CV_32F);
    return working;
  }

  cv::Mat background;
  cv::convertScaleAbs(background_gray_f32_, background);
  cv::Mat motion;
  cv::absdiff(gray, background, motion);
  cv::threshold(motion, motion, config_.motion_threshold, 255,
                cv::THRESH_BINARY);
  if (config_.motion_erode_px > 0) {
    if (erode_kernel_.empty()) {
      const auto size = 2 * config_.motion_erode_px + 1;
      erode_kernel_ =
          cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size));
    }
    cv::erode(motion, motion, erode_kernel_);
  }
  if (config_.motion_dilate_px > 0) {
    if (dilate_kernel_.empty()) {
      const auto size = 2 * config_.motion_dilate_px + 1;
      dilate_kernel_ =
          cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size));
    }
    cv::dilate(motion, motion, dilate_kernel_);
  }
  result.motion_ratio =
      static_cast<float>(static_cast<double>(cv::countNonZero(motion)) /
                         static_cast<double>(motion.total()));
  const auto alpha = std::clamp(config_.motion_ratio_ema_alpha, 0.01, 1.0);
  smoothed_motion_ratio_ =
      motion_initialized_
          ? smoothed_motion_ratio_ +
                alpha * (result.motion_ratio - smoothed_motion_ratio_)
          : result.motion_ratio;
  motion_initialized_ = true;
  if (global_motion_active_) {
    global_motion_active_ =
        smoothed_motion_ratio_ > config_.trail_resume_motion_ratio;
  } else {
    global_motion_active_ =
        smoothed_motion_ratio_ >= config_.trail_disable_motion_ratio;
  }
  result.suppress_trail = global_motion_active_;
  cv::accumulateWeighted(gray, background_gray_f32_,
                         std::clamp(config_.bg_update_alpha, 0.001, 0.2));
  if (global_motion_active_) {
    trail_masks_.clear();
    trail_frames_.clear();
    return working;
  }

  cv::Mat focused = working.clone();
  if (config_.static_simplify) {
    cv::Mat static_base;
    cv::cvtColor(working, static_base, cv::COLOR_BGR2GRAY);
    cv::cvtColor(static_base, static_base, cv::COLOR_GRAY2BGR);
    cv::GaussianBlur(static_base, focused, cv::Size(),
                     std::max(0.0, config_.bg_blur_sigma));
    cv::Mat focus_mask = motion.clone();
    const auto clear =
        std::min({config_.center_clear_size, focused.cols, focused.rows});
    if (clear > 0) {
      cv::rectangle(focus_mask,
                    cv::Rect((focused.cols - clear) / 2,
                             (focused.rows - clear) / 2, clear, clear),
                    cv::Scalar(255), cv::FILLED);
    }
    working.copyTo(focused, focus_mask);
  }
  if (config_.motion_trail_frames > 0) {
    trail_masks_.push_back(motion);
    trail_frames_.push_back(working);
    const auto max_history =
        static_cast<std::size_t>(config_.motion_trail_frames + 1);
    while (trail_masks_.size() > max_history) {
      trail_masks_.pop_front();
      trail_frames_.pop_front();
    }
    if (trail_masks_.size() > 1U) {
      cv::Mat trail_mask = motion.clone();
      cv::Mat trail_image = working.clone();
      for (std::size_t index = 0; index + 1U < trail_masks_.size(); ++index) {
        cv::bitwise_or(trail_mask, trail_masks_[index], trail_mask);
        cv::max(trail_image, trail_frames_[index], trail_image);
      }
      if (config_.trail_brightness_gain > 1.0) {
        // 与 26hero 一致：只提亮 Y，不能把 BGR 三通道同时放大而改变色相。
        cv::Mat ycrcb;
        cv::cvtColor(trail_image, ycrcb, cv::COLOR_BGR2YCrCb);
        std::vector<cv::Mat> channels;
        cv::split(ycrcb, channels);
        channels[0].convertTo(
            channels[0], CV_8U,
            std::clamp(config_.trail_brightness_gain, 1.0, 3.0));
        cv::merge(channels, ycrcb);
        cv::cvtColor(ycrcb, trail_image, cv::COLOR_YCrCb2BGR);
      }
      trail_image.copyTo(focused, trail_mask);
    }
  }
  return focused;
}

std::vector<Packet> AntiBaseProcessor::popPackets() {
  std::vector<Packet> result;
  const auto now = steadyNowNs();
  const auto gap_ns = static_cast<int64_t>(
      std::chrono::duration<double, std::milli>(config_.min_packet_gap_ms)
          .count() *
      1000000.0);
  if (next_packet_admit_ns_ == 0) {
    next_packet_admit_ns_ = now;
  }
  while (stream_buffer_.size() >= kPacketPayloadBytes &&
         now >= next_packet_admit_ns_) {
    Packet packet;
    packet.sequence_id = sequence_id_++;
    std::copy_n(stream_buffer_.begin(), kPacketPayloadBytes,
                packet.data.begin());
    stream_buffer_.erase(stream_buffer_.begin(),
                         stream_buffer_.begin() +
                             static_cast<std::ptrdiff_t>(kPacketPayloadBytes));
    result.push_back(packet);
    next_packet_admit_ns_ += std::max<int64_t>(1, gap_ns);
  }
  if (stream_buffer_.size() > max_backlog_bytes_) {
    auto drop = stream_buffer_.size() - max_backlog_bytes_;
    for (std::size_t index = drop; index + 4U < stream_buffer_.size();
         ++index) {
      const bool start3 = stream_buffer_[index] == 0U &&
                          stream_buffer_[index + 1U] == 0U &&
                          stream_buffer_[index + 2U] == 1U;
      const bool start4 = start3 || (stream_buffer_[index] == 0U &&
                                     stream_buffer_[index + 1U] == 0U &&
                                     stream_buffer_[index + 2U] == 0U &&
                                     stream_buffer_[index + 3U] == 1U);
      if (start4) {
        drop = index;
        break;
      }
    }
    dropped_bytes_ += drop;
    stream_buffer_.erase(stream_buffer_.begin(),
                         stream_buffer_.begin() +
                             static_cast<std::ptrdiff_t>(drop));
  }
  return result;
}

void AntiBaseProcessor::updateBitrate(const TxFeedback &feedback) {
  if (!config_.adaptive_bitrate_enabled) {
    return;
  }
  const auto now = steadyNowNs();
  if (last_control_ns_ == 0) {
    last_control_ns_ = now;
    last_feedback_ns_ = now;
    last_sent_packets_ = feedback.sent_packets;
    last_control_dropped_bytes_ = dropped_bytes_;
    return;
  }
  const auto interval_ns = static_cast<int64_t>(
      std::max(0.2, config_.adaptive_bitrate_control_interval_sec) * 1e9);
  if (now - last_control_ns_ < interval_ns) {
    return;
  }
  const auto elapsed_s = std::max(
      0.001, static_cast<double>(now - last_feedback_ns_) / 1e9);
  const auto sent_delta = feedback.sent_packets >= last_sent_packets_
                              ? feedback.sent_packets - last_sent_packets_
                              : 0U;
  const auto tx_pps = static_cast<double>(sent_delta) / elapsed_s;
  const auto encoded_kbps = static_cast<double>(encoded_bytes_since_control_) *
                            8.0 / 1000.0 / elapsed_s;
  const auto full_pps = 1000.0 / config_.min_packet_gap_ms;
  // H.264 可用净载荷不包含 8B 序号：292B * 49.75Hz = 116.2kbps。
  const auto h264_capacity_kbps =
      static_cast<double>(kPacketPayloadBytes) * 8.0 /
      config_.min_packet_gap_ms;
  const bool queue_hard =
      feedback.pending_packets >= 8U ||
      feedback.oldest_pending_ms >= config_.min_packet_gap_ms * 8.0;
  const bool queue_high =
      feedback.pending_packets >= 4U ||
      feedback.oldest_pending_ms >= config_.min_packet_gap_ms * 4.0;
  const bool limiter_hard = dropped_bytes_ > last_control_dropped_bytes_ ||
                            stream_buffer_.size() >=
                                kPacketPayloadBytes * 8U;
  const bool limiter_high =
      stream_buffer_.size() >= kPacketPayloadBytes * 4U;
  int next = current_bitrate_kbps_;
  if (queue_hard || limiter_hard) {
    next -= 2 * config_.adaptive_bitrate_step_down_kbps;
  } else if (queue_high || limiter_high) {
    next -= config_.adaptive_bitrate_step_down_kbps;
  } else if (tx_pps < full_pps * 0.95 &&
             encoded_kbps < h264_capacity_kbps * 0.98 &&
             feedback.pending_packets <= 1U &&
             feedback.oldest_pending_ms < config_.min_packet_gap_ms) {
    next += config_.adaptive_bitrate_step_up_kbps;
  }
  next = std::clamp(next, config_.adaptive_bitrate_min_kbps,
                    config_.adaptive_bitrate_max_kbps);
  if (next != current_bitrate_kbps_) {
    if (gst_->encoder != nullptr) {
      g_object_set(G_OBJECT(gst_->encoder), "bitrate", next, nullptr);
    }
    current_bitrate_kbps_ = next;
  }
  last_control_ns_ = now;
  last_feedback_ns_ = now;
  last_sent_packets_ = feedback.sent_packets;
  encoded_bytes_since_control_ = 0U;
  last_control_dropped_bytes_ = dropped_bytes_;
}

void AntiBaseProcessor::initializeEncoder() {
  gst_init(nullptr, nullptr);
  gst_->pipeline = gst_pipeline_new("hero_antibase_encoder");
  gst_->appsrc = gst_element_factory_make("appsrc", "source");
  auto *convert = gst_element_factory_make("videoconvert", "convert");
  gst_->encoder = gst_element_factory_make("x264enc", "encoder");
  auto *parser = gst_element_factory_make("h264parse", "parser");
  gst_->appsink = gst_element_factory_make("appsink", "sink");
  if (!gst_->pipeline || !gst_->appsrc || !convert || !gst_->encoder ||
      !gst_->appsink) {
    throw std::runtime_error("无法创建 GStreamer x264 反基地编码管线");
  }
  auto *input_caps = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, "BGR", "width", G_TYPE_INT,
      config_.output_size, "height", G_TYPE_INT, config_.output_size,
      "framerate", GST_TYPE_FRACTION, config_.output_fps, 1, nullptr);
  g_object_set(G_OBJECT(gst_->appsrc), "caps", input_caps, "stream-type", 0,
               "is-live", TRUE, "format", GST_FORMAT_TIME, "do-timestamp",
               TRUE, nullptr);
  gst_caps_unref(input_caps);
  g_object_set(G_OBJECT(gst_->encoder), "bitrate", current_bitrate_kbps_,
               "speed-preset", config_.h264_speed_preset, "tune",
               config_.h264_tune, "byte-stream", TRUE, "key-int-max",
               config_.h264_key_int_max, "bframes", config_.h264_bframes,
               "rc-lookahead", config_.h264_rc_lookahead, "sync-lookahead",
               config_.h264_sync_lookahead, "sliced-threads",
               config_.h264_sliced_threads, "ref", config_.h264_ref, "aud",
               TRUE, "vbv-buf-capacity", config_.h264_vbv_buf_capacity,
               "option-string", config_.h264_option_string.c_str(), nullptr);
  if (parser != nullptr) {
    g_object_set(G_OBJECT(parser), "config-interval", -1,
                 "disable-passthrough", TRUE, nullptr);
  }
  auto *h264_caps = gst_caps_new_simple(
      "video/x-h264", "stream-format", G_TYPE_STRING, "byte-stream",
      "alignment", G_TYPE_STRING, "au", nullptr);
  g_object_set(G_OBJECT(gst_->appsink), "caps", h264_caps, "max-buffers", 2,
               "drop", TRUE, "emit-signals", FALSE, "sync", FALSE, nullptr);
  gst_caps_unref(h264_caps);
  gst_bin_add_many(GST_BIN(gst_->pipeline), gst_->appsrc, convert,
                   gst_->encoder, nullptr);
  if (parser != nullptr) {
    gst_bin_add_many(GST_BIN(gst_->pipeline), parser, gst_->appsink, nullptr);
  } else {
    gst_bin_add_many(GST_BIN(gst_->pipeline), gst_->appsink, nullptr);
  }
  const bool linked = parser != nullptr
                          ? gst_element_link_many(gst_->appsrc, convert,
                                                  gst_->encoder, parser,
                                                  gst_->appsink, nullptr)
                          : gst_element_link_many(gst_->appsrc, convert,
                                                  gst_->encoder, gst_->appsink,
                                                  nullptr);
  if (!linked ||
      gst_element_set_state(gst_->pipeline, GST_STATE_PLAYING) ==
          GST_STATE_CHANGE_FAILURE) {
    throw std::runtime_error("无法启动 GStreamer x264 反基地编码管线");
  }
}

void AntiBaseProcessor::shutdownEncoder() {
  if (gst_ != nullptr && gst_->pipeline != nullptr) {
    gst_element_set_state(gst_->pipeline, GST_STATE_NULL);
    gst_object_unref(gst_->pipeline);
    gst_->pipeline = nullptr;
    gst_->appsrc = nullptr;
    gst_->appsink = nullptr;
    gst_->encoder = nullptr;
  }
}

std::vector<uint8_t> AntiBaseProcessor::encode(const cv::Mat &image) {
  std::vector<uint8_t> output;
  if (image.empty() || !image.isContinuous()) {
    return output;
  }
  const auto bytes = image.total() * image.elemSize();
  auto *buffer = gst_buffer_new_allocate(nullptr, bytes, nullptr);
  GstMapInfo input_map;
  if (buffer == nullptr || !gst_buffer_map(buffer, &input_map, GST_MAP_WRITE)) {
    if (buffer != nullptr) {
      gst_buffer_unref(buffer);
    }
    return output;
  }
  std::memcpy(input_map.data, image.data, bytes);
  gst_buffer_unmap(buffer, &input_map);
  if (gst_app_src_push_buffer(GST_APP_SRC(gst_->appsrc), buffer) !=
      GST_FLOW_OK) {
    return output;
  }
  bool first = true;
  while (true) {
    auto *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(gst_->appsink),
                                                first ? 5 * GST_MSECOND : 0);
    first = false;
    if (sample == nullptr) {
      break;
    }
    auto *encoded = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (encoded != nullptr && gst_buffer_map(encoded, &map, GST_MAP_READ)) {
      output.insert(output.end(), map.data, map.data + map.size);
      gst_buffer_unmap(encoded, &map);
    }
    gst_sample_unref(sample);
  }
  return output;
}

} // namespace hero_antibase
