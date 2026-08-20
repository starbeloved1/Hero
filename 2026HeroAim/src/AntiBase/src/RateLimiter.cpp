#include "AntiBase/include/RateLimiter.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <sstream>

namespace antibase {

int64_t RateLimiter::nowNs() {
    using Clock = std::chrono::steady_clock;
    static const auto kStart = Clock::now();
    auto elapsed = Clock::now() - kStart;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
}

RateLimiter::RateLimiter(
    double max_tx_delay_s,
    std::size_t packet_size,
    double min_packet_gap_ms)
    : bandwidth_limit_kbytes_(static_cast<double>(packet_size) /
                              std::max(20.001, min_packet_gap_ms)),
      bandwidth_window_s_(1.0),
      packet_size_(packet_size),
      packet_payload_size_(packet_size > Packetizer0310::kHeaderBytes
                               ? packet_size - Packetizer0310::kHeaderBytes
                               : 0),
      window_ns_(1000000000LL),
      window_limit_bytes_(0),
      max_backlog_bytes_(static_cast<std::size_t>(bandwidth_limit_kbytes_ * 1000.0 * max_tx_delay_s)),
      sent_window_bytes_(0),
      dropped_bytes_(0),
      dropped_events_(0),
      last_telemetry_ns_(0) {
    const double effective_window_bytes = bandwidth_limit_kbytes_ * 1000.0;
    window_limit_bytes_ = static_cast<std::size_t>(std::max(1.0, effective_window_bytes));
}

void RateLimiter::reset() {
    stream_buffer_.clear();
    sent_window_.clear();
    sent_window_bytes_ = 0;
    dropped_bytes_ = 0;
    dropped_events_ = 0;
    last_telemetry_ns_ = 0;
    last_clip_message_.clear();
}

void RateLimiter::appendEncoded(const std::vector<uint8_t>& encoded) {
    if (encoded.empty()) {
        return;
    }
    std::size_t old_size = stream_buffer_.size();
    stream_buffer_.resize(old_size + encoded.size());
    std::memcpy(stream_buffer_.data() + old_size, encoded.data(), encoded.size());
}

std::vector<AntiBasePacket> RateLimiter::popSendable(Packetizer0310& packetizer, uint64_t& packet_sequence_id) {
    std::vector<AntiBasePacket> packets;
    if (packet_size_ == 0 || packet_payload_size_ == 0) {
        return packets;
    }

    while (stream_buffer_.size() >= packet_payload_size_) {
        const int64_t now_ns = nowNs();
        while (!sent_window_.empty() && (now_ns - sent_window_.front().first) > window_ns_) {
            sent_window_bytes_ -= sent_window_.front().second;
            sent_window_.pop_front();
        }
        if (sent_window_bytes_ + packet_size_ > window_limit_bytes_) {
            break;
        }

        AntiBasePacket pkt = packetizer.makePacket(
            stream_buffer_.data(),
            packet_payload_size_,
            packet_sequence_id++);
        packets.emplace_back(std::move(pkt));

        sent_window_.emplace_back(now_ns, packet_size_);
        sent_window_bytes_ += packet_size_;

        std::memmove(
            stream_buffer_.data(),
            stream_buffer_.data() + packet_payload_size_,
            stream_buffer_.size() - packet_payload_size_);
        stream_buffer_.resize(stream_buffer_.size() - packet_payload_size_);
    }

    if (stream_buffer_.size() > max_backlog_bytes_) {
        const std::size_t target_drop = stream_buffer_.size() - max_backlog_bytes_;
        std::size_t drop_bytes = target_drop;

        for (std::size_t i = target_drop; i + 4 < stream_buffer_.size(); ++i) {
            const bool start_code_3 = (stream_buffer_[i] == 0 && stream_buffer_[i + 1] == 0 &&
                                       stream_buffer_[i + 2] == 1);
            const bool start_code_4 = (stream_buffer_[i] == 0 && stream_buffer_[i + 1] == 0 &&
                                       stream_buffer_[i + 2] == 0 && stream_buffer_[i + 3] == 1);
            if (start_code_3 || start_code_4) {
                drop_bytes = i;
                break;
            }
        }

        std::memmove(
            stream_buffer_.data(),
            stream_buffer_.data() + drop_bytes,
            stream_buffer_.size() - drop_bytes);
        stream_buffer_.resize(stream_buffer_.size() - drop_bytes);

        dropped_bytes_ += drop_bytes;
        dropped_events_++;

        std::ostringstream oss;
        oss << "传输积压已裁剪: 本次丢弃=" << drop_bytes
            << "B 当前积压=" << stream_buffer_.size()
            << "B 累计丢弃=" << dropped_bytes_
            << "B 触发次数=" << dropped_events_;
        last_clip_message_ = oss.str();
    }

    return packets;
}

TxStats RateLimiter::snapshotStats() const {
    TxStats s;
    s.window_kbytes = static_cast<double>(sent_window_bytes_) / 1000.0;
    s.window_limit_kbytes = static_cast<double>(window_limit_bytes_) / 1000.0;
    s.avg_kbytes_per_s = s.window_kbytes / bandwidth_window_s_;
    s.backlog_bytes = stream_buffer_.size();
    s.dropped_bytes = dropped_bytes_;
    s.dropped_events = dropped_events_;
    return s;
}

bool RateLimiter::shouldLogTelemetry() {
    const int64_t now_ns = nowNs();
    if (now_ns - last_telemetry_ns_ > 1000000000LL) {
        last_telemetry_ns_ = now_ns;
        return true;
    }
    return false;
}

std::string RateLimiter::lastClipMessage() {
    std::string msg = last_clip_message_;
    last_clip_message_.clear();  // 取出消息后清空，防止重复打印
    return msg;
}

}  // namespace antibase
