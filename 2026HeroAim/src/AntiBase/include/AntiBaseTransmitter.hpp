#ifndef HERO_26_ANTIBASE_TRANSMITTER_HPP
#define HERO_26_ANTIBASE_TRANSMITTER_HPP

#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>

#include "AntiBase/include/AntiBaseManager.hpp"
#include "Driver/include/SerialPort.hpp"

namespace antibase {

class AntiBaseTransmitter {
public:
    AntiBaseTransmitter();
    ~AntiBaseTransmitter();

    void send(const AntiBaseOutput& output, driver::SerialPort& serial_port);
    TxFeedback feedback() const;
    // 切换图像源时取消尚未开始发送的旧源整包。
    std::size_t clearPendingPackets();

private:
    struct PendingPacket {
        AntiBasePacket packet;
        driver::SerialPort* serial_port = nullptr;
        std::chrono::steady_clock::time_point enqueued_at{};
    };

    void initUdpDebug();
    void closeUdpDebug();
    void sendLoop();
    void sendUdpDebugPacket(const AntiBasePacket& packet);
    void sendSerialPacket(const AntiBasePacket& packet, driver::SerialPort& serial_port);
    void recordSendTelemetry(std::chrono::steady_clock::time_point sent_at);
    void logStats();

    int udp_fd_ = -1;
    bool udp_debug_ready_ = false;

    // 实际出链路的节拍必须独立于视频帧率；否则 30fps 编码会错误地把
    // 300B 包吞吐限制到 9kB/s。
    std::chrono::nanoseconds min_packet_gap_{0};
    std::thread send_thread_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<PendingPacket> pending_packets_;
    bool running_ = true;
    std::atomic<uint64_t> total_sent_packets_{0};

    std::mutex telemetry_mutex_;
    std::chrono::steady_clock::time_point last_send_time_{};
    std::chrono::steady_clock::time_point telemetry_start_{};
    uint64_t telemetry_packets_ = 0;
    uint64_t telemetry_gaps_ = 0;
    uint64_t telemetry_under_gap_ = 0;
    double telemetry_gap_sum_ms_ = 0.0;
    double telemetry_gap_min_ms_ = 0.0;
    double telemetry_gap_max_ms_ = 0.0;
};

}  // namespace antibase

#endif
