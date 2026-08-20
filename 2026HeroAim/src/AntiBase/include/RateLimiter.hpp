#ifndef HERO_26_ANTIBASE_RATELIMITER_HPP
#define HERO_26_ANTIBASE_RATELIMITER_HPP

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include "AntiBase/include/Packetizer0310.hpp"

namespace antibase {

struct TxStats {
    double window_kbytes = 0.0;
    double window_limit_kbytes = 0.0;
    double avg_kbytes_per_s = 0.0;
    std::size_t backlog_bytes = 0;
    uint64_t dropped_bytes = 0;
    uint32_t dropped_events = 0;
};

class RateLimiter {
public:
    RateLimiter(
        double max_tx_delay_s,
        std::size_t packet_size,
        double min_packet_gap_ms
    );

    void appendEncoded(const std::vector<uint8_t>& encoded);
    std::vector<AntiBasePacket> popSendable(Packetizer0310& packetizer, uint64_t& packet_sequence_id);
    // 切换相机源时丢弃旧码流和旧速率窗口，避免新编码器被旧数据阻塞。
    void reset();

    TxStats snapshotStats() const;
    bool shouldLogTelemetry();
    std::string lastClipMessage();  // 返回后自动清空消息

private:
    static int64_t nowNs();

    double bandwidth_limit_kbytes_;
    double bandwidth_window_s_;
    std::size_t packet_size_;
    std::size_t packet_payload_size_;

    std::vector<uint8_t> stream_buffer_;

    int64_t window_ns_;
    std::size_t window_limit_bytes_;
    std::size_t max_backlog_bytes_;

    std::deque<std::pair<int64_t, std::size_t>> sent_window_;
    std::size_t sent_window_bytes_;
    uint64_t dropped_bytes_;
    uint32_t dropped_events_;

    int64_t last_telemetry_ns_;
    std::string last_clip_message_;
};

}  // namespace antibase

#endif
