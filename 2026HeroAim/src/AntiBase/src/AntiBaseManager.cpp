#include "AntiBase/include/AntiBaseManager.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "utils/include/Log.hpp"
#include "utils/include/Params.hpp"

namespace antibase {

namespace {
using SteadyClock = std::chrono::steady_clock;
constexpr std::size_t kQueueHighPackets = 4;
constexpr std::size_t kQueueHardPackets = 8;
constexpr double kUnderfillRatio = 0.95;

double effectivePacketGapMs() {
    // 官方要求严格大于 20ms。即使配置误填，也不能放宽到不合规区间。
    return std::max(20.001, lyutils::AntiBaseParam::limiter_min_packet_gap_ms);
}

PreprocessConfig makePreprocessConfig() {
    PreprocessConfig cfg{};
    cfg.crop_size = lyutils::AntiBaseParam::crop_size;
    cfg.crop_offset_x = lyutils::AntiBaseParam::crop_offset_x;
    cfg.output_size = lyutils::AntiBaseParam::output_size;
    cfg.static_simplify = lyutils::AntiBaseParam::static_simplify;
    cfg.motion_threshold = lyutils::AntiBaseParam::motion_threshold;
    cfg.motion_erode_px = lyutils::AntiBaseParam::motion_erode_px;
    cfg.motion_dilate_px = lyutils::AntiBaseParam::motion_dilate_px;
    cfg.motion_trail_frames = lyutils::AntiBaseParam::motion_trail_frames;
    cfg.trail_brightness_gain = lyutils::AntiBaseParam::trail_brightness_gain;
    cfg.trail_disable_motion_ratio = lyutils::AntiBaseParam::trail_disable_motion_ratio;
    cfg.trail_resume_motion_ratio = lyutils::AntiBaseParam::trail_resume_motion_ratio;
    cfg.motion_ratio_ema_alpha = lyutils::AntiBaseParam::motion_ratio_ema_alpha;
    cfg.bg_update_alpha = lyutils::AntiBaseParam::bg_update_alpha;
    cfg.bg_blur_sigma = lyutils::AntiBaseParam::bg_blur_sigma;
    cfg.center_clear_size = lyutils::AntiBaseParam::center_clear_size;
    cfg.force_monochrome = lyutils::AntiBaseParam::force_monochrome;
    cfg.target_bitrate_kbps = lyutils::AntiBaseParam::target_bitrate_kbps;
    return cfg;
}

double elapsedMs(const SteadyClock::time_point& start, const SteadyClock::time_point& end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}
}

AntiBaseManager::AntiBaseManager()
    : output_fps_(lyutils::AntiBaseParam::output_fps),
    last_encode_stamp_ns_(0),
    packet_sequence_id_(0),
    current_bitrate_kbps_(lyutils::AntiBaseParam::target_bitrate_kbps),
    last_bitrate_control_ns_(0),
    last_tx_sample_ns_(0),
    last_tx_sent_packets_(0),
    encoded_bytes_since_control_(0),
    last_limiter_dropped_events_(0),
    last_rc_tx_pps_(0.0),
    last_rc_encoded_kbps_(0.0),
    last_rc_pending_packets_(0),
    last_rc_pending_age_ms_(0.0),
    last_rc_action_("warmup"),
    preprocess_cfg_(makePreprocessConfig()),
    preprocess_(preprocess_cfg_),
    packetizer_(static_cast<std::size_t>(std::max(0, lyutils::AntiBaseParam::packet_size))),
    rate_limiter_(
        lyutils::AntiBaseParam::max_tx_delay_s,
        packetizer_.packetSize(),
        effectivePacketGapMs())
#ifdef HAVE_GSTREAMER_X264
    , pipeline_(nullptr),
      appsrc_(nullptr),
      appsink_(nullptr),
      encoder_(nullptr),
      bus_(nullptr)
#endif
{
    const TxStats link_budget = rate_limiter_.snapshotStats();
    LOG(INFO) << "[M4 INIT] fps=" << output_fps_
              << " packet_size=" << packetizer_.packetSize()
              << " bw_limit(derived)=" << link_budget.window_limit_kbytes << "kB/s"
              << " min_gap_ms=" << effectivePacketGapMs()
              << " target_bitrate=" << current_bitrate_kbps_ << "kbps"
              << " adaptive_bitrate=" << lyutils::AntiBaseParam::adaptive_bitrate_enabled
              << " range=" << lyutils::AntiBaseParam::adaptive_bitrate_min_kbps
              << "-" << lyutils::AntiBaseParam::adaptive_bitrate_max_kbps << "kbps";
#ifdef HAVE_GSTREAMER_X264
    if (!initializeGstreamer()) {
        LOG(WARNING) << "AntiBase GStreamer 初始化失败，编码输出可能为空。";
    }
#endif
}

AntiBaseManager::~AntiBaseManager() {
#ifdef HAVE_GSTREAMER_X264
    shutdownGstreamer();
#endif
}

void AntiBaseManager::resetForSourceSwitch() {
    // 背景、拖影和静态简化都属于图像源的历史状态，不能跨相机复用。
    preprocess_ = Preprocess(preprocess_cfg_);
    rate_limiter_.reset();
    last_encode_stamp_ns_ = 0;
    last_bitrate_control_ns_ = 0;
    last_tx_sample_ns_ = 0;
    last_tx_sent_packets_ = 0;
    encoded_bytes_since_control_ = 0;
    last_limiter_dropped_events_ = 0;
    last_rc_tx_pps_ = 0.0;
    last_rc_encoded_kbps_ = 0.0;
    last_rc_pending_packets_ = 0;
    last_rc_pending_age_ms_ = 0.0;
    last_rc_action_ = "source_switch";

#ifdef HAVE_GSTREAMER_X264
    // 重新建 x264 管线会清掉参考帧；首个 AU 由新编码器以 IDR（含参数集）开始。
    shutdownGstreamer();
    if (!initializeGstreamer()) {
        LOG(WARNING) << "AntiBase source switch: GStreamer 重建失败，编码输出可能为空。";
    }
#endif
}

AntiBaseOutput AntiBaseManager::process(const cv::Mat& frame, const TxFeedback& tx_feedback) {
    const auto process_begin = SteadyClock::now();
    AntiBaseOutput output;
    updateAdaptiveBitrate(tx_feedback);
    if (frame.empty()) {
        return output;
    }

    // 编码可以抽帧，但不能因此停止从积压码流取包；真正的整包出包节拍
    // 由 AntiBaseTransmitter 的发送线程保证。
    bool encode_due = true;
    if (output_fps_ <= 60) {
        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::steady_clock::now().time_since_epoch())
                                 .count();
        const int64_t frame_interval_ns = 1000000000LL / std::max(output_fps_, 1);
        // 相机硬件固定在与输出相同的帧率时，调度/USB 抖动会让相邻帧偶尔
        // 早于理论周期几十到几千微秒。给 5% 容差，避免 39ms 的帧被跳过后
        // 变成下一帧 80ms 才编码的明显卡顿。
        const int64_t early_tolerance_ns = frame_interval_ns / 20;
        if (last_encode_stamp_ns_ > 0 &&
            (now_ns - last_encode_stamp_ns_) < (frame_interval_ns - early_tolerance_ns)) {
            encode_due = false;
        } else {
            last_encode_stamp_ns_ = now_ns;
        }
    }

    if (!encode_due) {
        output.packets = rate_limiter_.popSendable(packetizer_, packet_sequence_id_);
        output.rate_limited = output.packets.empty();
        return output;
    }

    const auto preprocess_begin = SteadyClock::now();
    //preprocess处理
    PreprocessResult prep = preprocess_.run(frame);
    const auto preprocess_end = SteadyClock::now();
    output.visual_frame = prep.final_frame;
    output.roi_frame = prep.roi_downsample;
    output.static_frame = prep.static_removed;
    output.motion_ratio = prep.motion_ratio;
    output.suppress_trail = prep.suppress_trail;

    if (output.visual_frame.empty()) {
        output.rate_limited = true;
        return output;
    }

    const auto encode_begin = SteadyClock::now();
    //编码与流缓冲分包（逻辑停在拿包，不发送）
    std::vector<uint8_t> encoded = encodeFrame(output.visual_frame);
    const auto encode_end = SteadyClock::now();
    output.encoded_bytes = encoded.size();
    encoded_bytes_since_control_ += encoded.size();

    const auto limiter_begin = SteadyClock::now();
    rate_limiter_.appendEncoded(encoded);
    output.packets = rate_limiter_.popSendable(packetizer_, packet_sequence_id_);
    const auto limiter_end = SteadyClock::now();

    if (output.packets.empty()) {
        output.rate_limited = true;
    }

    //统计信息
    const std::string clip_msg = rate_limiter_.lastClipMessage();
    if (!clip_msg.empty()) {
        LOG(WARNING) << "[M4 LIMIT] " << clip_msg;
    }

    if (rate_limiter_.shouldLogTelemetry()) {
        TxStats tx = rate_limiter_.snapshotStats();
        // 正常运行只保留这一条编码/准入汇总；发送侧另有一条 [M4 TX]。
        const double preprocess_ms = elapsedMs(preprocess_begin, preprocess_end);
        const double encode_ms = elapsedMs(encode_begin, encode_end);
        const double limiter_ms = elapsedMs(limiter_begin, limiter_end);
        const double process_total_ms = elapsedMs(process_begin, SteadyClock::now());

        std::ostringstream summary;
        summary << std::fixed << std::setprecision(2)
                << "[M4 ENC] rc=" << last_rc_action_
                << " target=" << current_bitrate_kbps_ << "kbps"
                << " enc=" << last_rc_encoded_kbps_ << "kbps"
                << " tx=" << last_rc_tx_pps_ << "pps"
                << " tx_q=" << last_rc_pending_packets_ << "/" << last_rc_pending_age_ms_ << "ms"
                << " motion=" << output.motion_ratio
                << " frame=" << output.encoded_bytes << "B/" << output.packets.size() << "pkt"
                << " admit=" << tx.window_kbytes << "/" << tx.window_limit_kbytes << "kB"
                << " raw_q=" << tx.backlog_bytes << "B"
                << " drop=" << tx.dropped_events
                << " cost=" << preprocess_ms << "+" << encode_ms << "+" << limiter_ms
                << "=" << process_total_ms << "ms";
        LOG(INFO) << summary.str();
    }

    return output;
}

void AntiBaseManager::updateAdaptiveBitrate(const TxFeedback& tx_feedback) {
    if (!lyutils::AntiBaseParam::adaptive_bitrate_enabled) {
        return;
    }

    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               SteadyClock::now().time_since_epoch()).count();
    if (last_tx_sample_ns_ == 0) {
        last_tx_sample_ns_ = now_ns;
        last_bitrate_control_ns_ = now_ns;
        last_tx_sent_packets_ = tx_feedback.sent_packets;
        last_limiter_dropped_events_ = rate_limiter_.snapshotStats().dropped_events;
        return;
    }

    const int64_t interval_ns = static_cast<int64_t>(
        std::max(0.2, lyutils::AntiBaseParam::adaptive_bitrate_control_interval_s) * 1e9);
    if (now_ns - last_bitrate_control_ns_ < interval_ns) {
        return;
    }

    const double elapsed_s = std::max(0.001, static_cast<double>(now_ns - last_tx_sample_ns_) / 1e9);
    const uint64_t sent_delta = tx_feedback.sent_packets >= last_tx_sent_packets_
        ? tx_feedback.sent_packets - last_tx_sent_packets_
        : 0;
    const double tx_pps = static_cast<double>(sent_delta) / elapsed_s;
    const double encoded_kbps = static_cast<double>(encoded_bytes_since_control_) * 8.0 / 1000.0 / elapsed_s;
    const TxStats limiter_stats = rate_limiter_.snapshotStats();

    const int min_kbps = std::max(1, lyutils::AntiBaseParam::adaptive_bitrate_min_kbps);
    const int max_kbps = std::max(min_kbps, lyutils::AntiBaseParam::adaptive_bitrate_max_kbps);
    const int step_up = std::max(1, lyutils::AntiBaseParam::adaptive_bitrate_step_up_kbps);
    const int step_down = std::max(1, lyutils::AntiBaseParam::adaptive_bitrate_step_down_kbps);
    const double packet_gap_ms = effectivePacketGapMs();
    const double full_pps = 1000.0 / packet_gap_ms;
    const bool queue_hard =
        tx_feedback.pending_packets >= kQueueHardPackets ||
        tx_feedback.oldest_pending_ms >= packet_gap_ms * kQueueHardPackets;
    const bool queue_high =
        tx_feedback.pending_packets >= kQueueHighPackets ||
        tx_feedback.oldest_pending_ms >= packet_gap_ms * kQueueHighPackets;
    const std::size_t payload_bytes = std::max<std::size_t>(1, packetizer_.payloadSize());
    const bool limiter_hard =
        limiter_stats.dropped_events > last_limiter_dropped_events_ ||
        limiter_stats.backlog_bytes >= payload_bytes * 8;
    const bool limiter_high = limiter_stats.backlog_bytes >= payload_bytes * 4;
    const double h264_capacity_kbps = limiter_stats.window_limit_kbytes *
        (static_cast<double>(payload_bytes) / std::max<std::size_t>(1, packetizer_.packetSize())) * 8.0;

    int next_bitrate = current_bitrate_kbps_;
    const char* reason = "hold";
    if (queue_hard || limiter_hard) {
        next_bitrate -= 2 * step_down;
        reason = queue_hard ? "queue_hard" : "limiter_hard";
    } else if (queue_high || limiter_high) {
        next_bitrate -= step_down;
        reason = queue_high ? "queue_high" : "limiter_backlog";
    } else if (tx_pps < full_pps * kUnderfillRatio &&
               encoded_kbps < h264_capacity_kbps * 0.98 &&
               tx_feedback.pending_packets <= 1 && tx_feedback.oldest_pending_ms < packet_gap_ms) {
        next_bitrate += step_up;
        reason = "underfill";
    }
    next_bitrate = std::clamp(next_bitrate, min_kbps, max_kbps);

#ifdef HAVE_GSTREAMER_X264
    if (next_bitrate != current_bitrate_kbps_) {
        setEncoderBitrate(next_bitrate);
        current_bitrate_kbps_ = next_bitrate;
    }
#endif

    last_rc_tx_pps_ = tx_pps;
    last_rc_encoded_kbps_ = encoded_kbps;
    last_rc_pending_packets_ = tx_feedback.pending_packets;
    last_rc_pending_age_ms_ = tx_feedback.oldest_pending_ms;
    last_rc_action_ = reason;

    last_bitrate_control_ns_ = now_ns;
    last_tx_sample_ns_ = now_ns;
    last_tx_sent_packets_ = tx_feedback.sent_packets;
    encoded_bytes_since_control_ = 0;
    last_limiter_dropped_events_ = limiter_stats.dropped_events;
}

std::vector<uint8_t> AntiBaseManager::encodeFrame(const cv::Mat& frame) const {
#ifdef HAVE_GSTREAMER_X264
    return encodeH264Frame(frame);
#endif
}

#ifdef HAVE_GSTREAMER_X264
bool AntiBaseManager::initializeGstreamer() {
    gst_init(nullptr, nullptr);

    pipeline_ = gst_pipeline_new("antibase_encoder_pipe");
    appsrc_ = gst_element_factory_make("appsrc", "source");
    appsink_ = gst_element_factory_make("appsink", "sink");
    GstElement* convert = gst_element_factory_make("videoconvert", "convert");
    encoder_ = gst_element_factory_make("x264enc", "encoder");
    GstElement* parser = gst_element_factory_make("h264parse", "parser");

    if (!pipeline_ || !appsrc_ || !appsink_ || !convert || !encoder_ || !parser) {
        return false;
    }

    GstCaps* caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        "width", G_TYPE_INT, preprocess_cfg_.output_size,
        "height", G_TYPE_INT, preprocess_cfg_.output_size,
        "framerate", GST_TYPE_FRACTION, output_fps_, 1,
        nullptr);
    g_object_set(
        G_OBJECT(appsrc_),
        "caps", caps,
        "stream-type", 0,
        "format", GST_FORMAT_TIME,
        "is-live", TRUE,
        "do-timestamp", TRUE,
        nullptr);
    gst_caps_unref(caps);

    const int key_int = lyutils::AntiBaseParam::h264_key_int_max > 0
        ? lyutils::AntiBaseParam::h264_key_int_max
        : std::max(output_fps_, 30);
    g_object_set(
        G_OBJECT(encoder_),
        "bitrate", current_bitrate_kbps_,
        "speed-preset", lyutils::AntiBaseParam::h264_speed_preset,
        "tune", lyutils::AntiBaseParam::h264_tune,
        "byte-stream", TRUE,
        "key-int-max", key_int,
        "bframes", lyutils::AntiBaseParam::h264_bframes,
        "rc-lookahead", lyutils::AntiBaseParam::h264_rc_lookahead,
        "sync-lookahead", lyutils::AntiBaseParam::h264_sync_lookahead,
        "sliced-threads", lyutils::AntiBaseParam::h264_sliced_threads,
        "ref", lyutils::AntiBaseParam::h264_ref,
        "aud", TRUE,
        "vbv-buf-capacity", lyutils::AntiBaseParam::h264_vbv_buf_capacity,
        "option-string", lyutils::AntiBaseParam::h264_option_string.c_str(),
        "pass", 0,
        nullptr);

    g_object_set(
        G_OBJECT(parser),
        "config-interval", -1,
        "disable-passthrough", TRUE,
        nullptr);

    GstCaps* h264_caps = gst_caps_new_simple(
        "video/x-h264",
        "stream-format", G_TYPE_STRING, "byte-stream",
        "alignment", G_TYPE_STRING, "au",
        nullptr);
    g_object_set(
        G_OBJECT(appsink_),
        "caps", h264_caps,
        "max-buffers", 2,
        "drop", TRUE,
        "emit-signals", FALSE,
        "sync", FALSE,
        nullptr);
    gst_caps_unref(h264_caps);

    gst_bin_add_many(GST_BIN(pipeline_), appsrc_, convert, encoder_, parser, appsink_, nullptr);
    if (!gst_element_link_many(appsrc_, convert, encoder_, parser, appsink_, nullptr)) {
        return false;
    }

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        return false;
    }

    bus_ = gst_element_get_bus(pipeline_);
    LOG(INFO) << "AntiBase GStreamer x264 编码器已就绪。"
              << " key_int=" << key_int
              << " bframes=" << lyutils::AntiBaseParam::h264_bframes
              << " ref=" << lyutils::AntiBaseParam::h264_ref
              << " rc_lookahead=" << lyutils::AntiBaseParam::h264_rc_lookahead
              << " sync_lookahead=" << lyutils::AntiBaseParam::h264_sync_lookahead
              << " sliced_threads=" << lyutils::AntiBaseParam::h264_sliced_threads
              << " vbv_buf_cap=" << lyutils::AntiBaseParam::h264_vbv_buf_capacity;
    return true;
}

void AntiBaseManager::setEncoderBitrate(int bitrate_kbps) {
    if (!encoder_) {
        return;
    }
    // x264enc 的 bitrate 属性可在 PLAYING 状态下修改；不重建管线，避免丢帧。
    g_object_set(G_OBJECT(encoder_), "bitrate", bitrate_kbps, nullptr);
}

void AntiBaseManager::shutdownGstreamer() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (bus_) {
        gst_object_unref(bus_);
        bus_ = nullptr;
    }
    if (pipeline_) {
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
    appsrc_ = nullptr;
    appsink_ = nullptr;
    encoder_ = nullptr;
}

std::vector<uint8_t> AntiBaseManager::encodeH264Frame(const cv::Mat& frame) const {
    std::vector<uint8_t> out;
    if (!appsrc_ || !appsink_ || frame.empty() || !frame.isContinuous()) {
        return out;
    }

    const std::size_t bytes = frame.total() * frame.elemSize();
    GstBuffer* gst_buffer = gst_buffer_new_allocate(nullptr, bytes, nullptr);
    if (!gst_buffer) {
        return out;
    }

    GstMapInfo map_info;
    if (!gst_buffer_map(gst_buffer, &map_info, GST_MAP_WRITE)) {
        gst_buffer_unref(gst_buffer);
        return out;
    }
    std::memcpy(map_info.data, frame.data, bytes);
    gst_buffer_unmap(gst_buffer, &map_info);

    const GstFlowReturn push_ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), gst_buffer);
    if (push_ret != GST_FLOW_OK) {
        return out;
    }

    bool first_pull = true;
    while (true) {
        GstSample* sample = gst_app_sink_try_pull_sample(
            GST_APP_SINK(appsink_),
            first_pull ? 5 * GST_MSECOND : 0);
        first_pull = false;
        if (!sample) {
            break;
        }

        GstBuffer* encoded = gst_sample_get_buffer(sample);
        if (encoded) {
            GstMapInfo enc_map;
            if (gst_buffer_map(encoded, &enc_map, GST_MAP_READ)) {
                const std::size_t old_size = out.size();
                out.resize(old_size + enc_map.size);
                std::memcpy(out.data() + old_size, enc_map.data, enc_map.size);
                gst_buffer_unmap(encoded, &enc_map);
            }
        }
        gst_sample_unref(sample);
    }

    return out;
}
#endif

}  // namespace antibase
