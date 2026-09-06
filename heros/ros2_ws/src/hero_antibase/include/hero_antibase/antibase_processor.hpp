#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace hero_antibase {

struct TxFeedback {
  uint64_t sent_packets{0U};
  uint32_t pending_packets{0U};
  float oldest_pending_ms{0.0F};
};

// 参数
struct AntiBaseConfig {
  int output_fps{30};
  int crop_size{960};
  int crop_offset_x{-120};
  int output_size{320};
  bool static_simplify{true};
  int motion_threshold{12};
  int motion_erode_px{1};
  int motion_dilate_px{2};
  int motion_trail_frames{4};
  double trail_brightness_gain{1.0};
  double trail_disable_motion_ratio{0.35};
  double trail_resume_motion_ratio{0.25};
  double motion_ratio_ema_alpha{0.20};
  double bg_update_alpha{0.01};
  double bg_blur_sigma{1.2};
  int center_clear_size{160};
  bool force_monochrome{false};
  int target_bitrate_kbps{130};
  double max_tx_delay_sec{0.5};
  double min_packet_gap_ms{22.0};
  bool adaptive_bitrate_enabled{true};
  int adaptive_bitrate_min_kbps{90};
  int adaptive_bitrate_max_kbps{160};
  int adaptive_bitrate_step_up_kbps{3};
  int adaptive_bitrate_step_down_kbps{10};
  double adaptive_bitrate_control_interval_sec{1.0};
  int h264_speed_preset{9};
  int h264_tune{4};
  int h264_key_int_max{35};
  int h264_bframes{0};
  int h264_rc_lookahead{0};
  int h264_sync_lookahead{0};
  bool h264_sliced_threads{true};
  int h264_ref{1};
  int h264_vbv_buf_capacity{80};
  std::string h264_option_string{
      "repeat-headers=1:scenecut=0:intra-refresh=0:open-gop=0:aq-mode=2:aq-"
      "strength=1.0:mbtree=1:qcomp=0.72:deblock=1,1:force-cfr=1"};
};

struct Packet {
  uint64_t sequence_id{0U};
  std::array<uint8_t, 292U> data{};
};

struct ProcessorResult {
  cv::Mat visualization;
  std::vector<Packet> packets;
  uint32_t encoded_bytes{0U};
  float motion_ratio{0.0F};
  bool suppress_trail{false};
  int bitrate_kbps{0};
  uint32_t raw_backlog_bytes{0U};
  uint64_t dropped_bytes{0U};
};

// 保留预处理、H.264码流缓冲、292B分包和自适应码率逻辑
class AntiBaseProcessor {
public:
  explicit AntiBaseProcessor(AntiBaseConfig config);
  ~AntiBaseProcessor();

  AntiBaseProcessor(const AntiBaseProcessor &) = delete;
  AntiBaseProcessor &operator=(const AntiBaseProcessor &) = delete;
  
  // antibase 总处理入口：
  // 1. 根据 output_fps 决定本次是否编码；未到编码时刻仍调用 popPackets() 取出已有码流包；
  // 2. preprocess()完成裁剪、缩放、背景/运动检测、静态简化和拖影预处理
  // 3. encode()使用 x264 编码为 H.264 字节流，追加至内部码流缓存；
  // 4. popPackets()按串口可用带宽将缓存切为 292B 数据包，并在积压超限时裁剪旧码流；
  // 5. updateBitrate()依据 gimbal_driver 的发送队列反馈调节下一周期编码码率。
  ProcessorResult process(const cv::Mat &image, const TxFeedback &feedback);
  void reset();

private:
  cv::Mat preprocess(const cv::Mat &image, ProcessorResult &result); // 预处理器
  std::vector<uint8_t> encode(const cv::Mat &image); // 编码器
  std::vector<Packet> popPackets(); // 限速器
  void updateBitrate(const TxFeedback &feedback); // 码率调整器
  void initializeEncoder();
  void shutdownEncoder();

  AntiBaseConfig config_;
  cv::Mat background_gray_f32_;
  cv::Mat erode_kernel_;
  cv::Mat dilate_kernel_;
  std::deque<cv::Mat> trail_masks_;
  std::deque<cv::Mat> trail_frames_;
  double smoothed_motion_ratio_{0.0};
  bool motion_initialized_{false};
  bool global_motion_active_{false};
  std::vector<uint8_t> stream_buffer_;
  std::deque<std::pair<int64_t, std::size_t>> send_window_;
  std::size_t send_window_bytes_{0U};
  std::size_t max_backlog_bytes_{0U};
  std::size_t window_limit_bytes_{0U};
  uint64_t dropped_bytes_{0U};
  uint64_t sequence_id_{0U};
  int current_bitrate_kbps_{0};
  int64_t last_encode_ns_{0};
  int64_t last_control_ns_{0};
  int64_t last_feedback_ns_{0};
  uint64_t last_sent_packets_{0U};
  uint64_t encoded_bytes_since_control_{0U};

  struct GstState;
  GstState *gst_{nullptr};
};

} // namespace hero_antibase
