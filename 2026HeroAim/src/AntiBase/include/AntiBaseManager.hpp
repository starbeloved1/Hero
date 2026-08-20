#ifndef HERO_26_ANTIBASE_MANAGER_HPP
#define HERO_26_ANTIBASE_MANAGER_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "AntiBase/include/Packetizer0310.hpp"
#include "AntiBase/include/Preprocess.hpp"
#include "AntiBase/include/RateLimiter.hpp"

#ifndef HAVE_GSTREAMER_X264
#error "AntiBase requires H264 encoder. Build with GStreamer and HAVE_GSTREAMER_X264."
#endif

#ifdef HAVE_GSTREAMER_X264
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#endif

namespace antibase {

struct AntiBaseOutput {
    cv::Mat visual_frame;
    cv::Mat roi_frame;
    cv::Mat static_frame;
    std::vector<AntiBasePacket> packets;
    bool rate_limited = false;
    std::size_t encoded_bytes = 0;
    double motion_ratio = 0.0;
    bool suppress_trail = false;
};

// 发送线程给编码控制器的真实链路状态。计数只在一整个 300B 逻辑包实际
// 开始发送时增加；队列时间因此包含了尚未下发到串口的所有等待时间。
struct TxFeedback {
    uint64_t sent_packets = 0;
    std::size_t pending_packets = 0;
    double oldest_pending_ms = 0.0;
};

class AntiBaseManager {
public:
    AntiBaseManager();
    ~AntiBaseManager();

    AntiBaseOutput process(const cv::Mat& frame, const TxFeedback& tx_feedback);
    // 输入相机切换栅栏：下一帧必从全新的预处理和 H.264 码流开始。
    void resetForSourceSwitch();

private:
    std::vector<uint8_t> encodeFrame(const cv::Mat& frame) const;
    void updateAdaptiveBitrate(const TxFeedback& tx_feedback);

#ifdef HAVE_GSTREAMER_X264
    bool initializeGstreamer();
    void shutdownGstreamer();
    void setEncoderBitrate(int bitrate_kbps);
    std::vector<uint8_t> encodeH264Frame(const cv::Mat& frame) const;
#endif

private:
    int output_fps_;
    int64_t last_encode_stamp_ns_;
    uint64_t packet_sequence_id_;
    int current_bitrate_kbps_;
    int64_t last_bitrate_control_ns_;
    int64_t last_tx_sample_ns_;
    uint64_t last_tx_sent_packets_;
    uint64_t encoded_bytes_since_control_;
    uint32_t last_limiter_dropped_events_;
    double last_rc_tx_pps_;
    double last_rc_encoded_kbps_;
    std::size_t last_rc_pending_packets_;
    double last_rc_pending_age_ms_;
    std::string last_rc_action_;

    PreprocessConfig preprocess_cfg_;
    Preprocess preprocess_;
    Packetizer0310 packetizer_;
    RateLimiter rate_limiter_;

#ifdef HAVE_GSTREAMER_X264
    GstElement* pipeline_;
    GstElement* appsrc_;
    GstElement* appsink_;
    GstElement* encoder_;
    GstBus* bus_;
#endif
};

}  // namespace antibase

#endif
