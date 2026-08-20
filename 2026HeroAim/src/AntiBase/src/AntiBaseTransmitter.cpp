#include "AntiBase/include/AntiBaseTransmitter.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "utils/include/Log.hpp"
#include "utils/include/Params.hpp"

namespace antibase {

namespace {

constexpr uint8_t kAntiBaseSof = '#';
constexpr std::size_t kAntiBaseDataBytes = 300;
constexpr std::size_t kChunkDataBytes = 60;
constexpr std::size_t kChunkCount = kAntiBaseDataBytes / kChunkDataBytes;
constexpr std::size_t kChunkHeaderBytes = 2;
constexpr std::size_t kChunkCrcBytes = 2;
constexpr std::size_t kChunkFrameBytes = kChunkHeaderBytes + kChunkDataBytes + kChunkCrcBytes;

std::string bytesToHex(const uint8_t* data, std::size_t len)
{
    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < len; ++i) {
        if (i > 0) {
            oss << ' ';
        }
        oss << std::setw(2) << static_cast<int>(data[i]);
    }
    return oss.str();
}

std::array<uint8_t, kAntiBaseDataBytes> makeJudgePayload(const AntiBasePacket& packet)
{
    std::array<uint8_t, kAntiBaseDataBytes> payload{};
    std::memcpy(payload.data(), &packet.sequence_id, sizeof(packet.sequence_id));

    const std::size_t payload_size = std::min(
        packet.data.size(),
        kAntiBaseDataBytes - sizeof(packet.sequence_id));
    if (payload_size > 0) {
        std::copy(
            packet.data.begin(),
            packet.data.begin() + static_cast<std::ptrdiff_t>(payload_size),
            payload.begin() + static_cast<std::ptrdiff_t>(sizeof(packet.sequence_id)));
    }

    return payload;
}

}  // namespace

AntiBaseTransmitter::AntiBaseTransmitter()
{
    initUdpDebug();
    // 该节拍是官方整包间隔的最终执行点，不能小于 20ms。
    const double configured_gap_ms = std::max(20.001, lyutils::AntiBaseParam::limiter_min_packet_gap_ms);
    min_packet_gap_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double, std::milli>(configured_gap_ms));
    telemetry_start_ = std::chrono::steady_clock::now();
    send_thread_ = std::thread(&AntiBaseTransmitter::sendLoop, this);
    LOG(INFO) << "[M4 TX INIT] min_gap=" << configured_gap_ms
              << "ms (official requirement: >20ms)";
}

AntiBaseTransmitter::~AntiBaseTransmitter()
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        running_ = false;
        pending_packets_.clear();
    }
    queue_cv_.notify_all();
    if (send_thread_.joinable()) {
        send_thread_.join();
    }
    closeUdpDebug();
}

void AntiBaseTransmitter::send(const AntiBaseOutput& output, driver::SerialPort& serial_port)
{
    if (output.packets.empty()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        const auto enqueued_at = std::chrono::steady_clock::now();
        for (const auto& packet : output.packets) {
            pending_packets_.push_back(PendingPacket{packet, &serial_port, enqueued_at});
        }
    }
    queue_cv_.notify_one();
}

TxFeedback AntiBaseTransmitter::feedback() const
{
    TxFeedback result;
    result.sent_packets = total_sent_packets_.load(std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(queue_mutex_);
    result.pending_packets = pending_packets_.size();
    if (!pending_packets_.empty()) {
        result.oldest_pending_ms = std::max(
            0.0,
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - pending_packets_.front().enqueued_at).count());
    }
    return result;
}

std::size_t AntiBaseTransmitter::clearPendingPackets()
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const std::size_t cleared = pending_packets_.size();
    pending_packets_.clear();
    return cleared;
}

void AntiBaseTransmitter::initUdpDebug()
{
    if (!lyutils::AntiBaseParam::udp_debug_enabled) {
        return;
    }

    udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd_ < 0) {
        LOG(ERROR) << "[AntiBase UDP Debug] Failed to create UDP socket.";
        return;
    }

    udp_debug_ready_ = true;
}

void AntiBaseTransmitter::closeUdpDebug()
{
    if (udp_fd_ >= 0) {
        ::close(udp_fd_);
        udp_fd_ = -1;
    }
    udp_debug_ready_ = false;
}

void AntiBaseTransmitter::sendLoop()
{
    while (true) {
        PendingPacket pending;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return !running_ || !pending_packets_.empty(); });
            if (!running_) {
                return;
            }

            if (last_send_time_ != std::chrono::steady_clock::time_point{} && min_packet_gap_.count() > 0) {
                const auto due = last_send_time_ + min_packet_gap_;
                const auto now = std::chrono::steady_clock::now();
                if (now < due) {
                    queue_cv_.wait_until(lock, due, [this] { return !running_; });
                    continue;
                }
            }

            pending = std::move(pending_packets_.front());
            pending_packets_.pop_front();
        }

        const auto sent_at = std::chrono::steady_clock::now();
        // 出包间隔以 300B 包真正开始出链路的时刻为准。串口分片会占用约
        // 10ms；若把它计入“上一包之后再等 21ms”，实际会被无谓拉到约 31ms。
        // 先记录起始时刻，下一包只补足到 21ms，串口耗时仍自然包含在间隔内。
        sendUdpDebugPacket(pending.packet);
        recordSendTelemetry(sent_at);
        total_sent_packets_.fetch_add(1, std::memory_order_relaxed);
        if (pending.serial_port != nullptr) {
            sendSerialPacket(pending.packet, *pending.serial_port);
        }
    }
}

void AntiBaseTransmitter::sendUdpDebugPacket(const AntiBasePacket& packet)
{
    if (!udp_debug_ready_ || udp_fd_ < 0) {
        return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(lyutils::AntiBaseParam::udp_debug_port));
    if (::inet_pton(AF_INET, lyutils::AntiBaseParam::udp_debug_host.c_str(), &addr.sin_addr) != 1) {
        LOG(ERROR) << "[AntiBase UDP Debug] Invalid host: " << lyutils::AntiBaseParam::udp_debug_host;
        udp_debug_ready_ = false;
        return;
    }

    const auto payload = makeJudgePayload(packet);
    const ssize_t sent = ::sendto(
        udp_fd_,
        payload.data(),
        payload.size(),
        0,
        reinterpret_cast<const sockaddr*>(&addr),
        sizeof(addr));
    if (sent != static_cast<ssize_t>(payload.size())) {
        LOG(ERROR) << "[AntiBase UDP Debug] sendto failed: " << std::strerror(errno);
    }
}

void AntiBaseTransmitter::sendSerialPacket(const AntiBasePacket& packet, driver::SerialPort& serial_port)
{
    const auto payload = makeJudgePayload(packet);

    if (lyutils::AntiBaseParam::log_packet_content) {
        LOG(INFO) << "[AntiBase 300B] seq=" << packet.sequence_id
                  << " len=" << payload.size()
                  << " data=" << bytesToHex(payload.data(), payload.size());
    }

    for (std::size_t chunk_index = 0; chunk_index < kChunkCount; ++chunk_index) {
        std::array<uint8_t, kChunkFrameBytes> chunk{};
        chunk[0] = kAntiBaseSof;
        chunk[1] = static_cast<uint8_t>(chunk_index);

        const std::size_t payload_offset = chunk_index * kChunkDataBytes;
        std::copy(
            payload.begin() + static_cast<std::ptrdiff_t>(payload_offset),
            payload.begin() + static_cast<std::ptrdiff_t>(payload_offset + kChunkDataBytes),
            chunk.begin() + static_cast<std::ptrdiff_t>(kChunkHeaderBytes));

        const uint16_t crc = driver::calculateCRC16_Frame(
            chunk.data(),
            static_cast<uint16_t>(kChunkHeaderBytes + kChunkDataBytes));
        chunk[chunk.size() - 2] = static_cast<uint8_t>(crc & 0xFF);
        chunk[chunk.size() - 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
        serial_port.serialPortWrite(chunk.data(), static_cast<int>(chunk.size()));
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void AntiBaseTransmitter::recordSendTelemetry(std::chrono::steady_clock::time_point sent_at)
{
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    if (last_send_time_ != std::chrono::steady_clock::time_point{}) {
        const double gap_ms = std::chrono::duration<double, std::milli>(sent_at - last_send_time_).count();
        ++telemetry_gaps_;
        telemetry_gap_sum_ms_ += gap_ms;
        telemetry_gap_min_ms_ = telemetry_gap_min_ms_ == 0.0 ? gap_ms : std::min(telemetry_gap_min_ms_, gap_ms);
        telemetry_gap_max_ms_ = std::max(telemetry_gap_max_ms_, gap_ms);
        if (gap_ms <= 20.0) {
            ++telemetry_under_gap_;
        }
    }
    last_send_time_ = sent_at;
    ++telemetry_packets_;

    if (sent_at - telemetry_start_ >= std::chrono::seconds(1)) {
        logStats();
        telemetry_start_ = sent_at;
        telemetry_packets_ = 0;
        telemetry_gaps_ = 0;
        telemetry_under_gap_ = 0;
        telemetry_gap_sum_ms_ = 0.0;
        telemetry_gap_min_ms_ = 0.0;
        telemetry_gap_max_ms_ = 0.0;
    }
}

void AntiBaseTransmitter::logStats()
{
    std::size_t queued = 0;
    {
        std::lock_guard<std::mutex> queue_lock(queue_mutex_);
        queued = pending_packets_.size();
    }
    const double elapsed_s = std::max(
        0.001,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - telemetry_start_).count());
    const double pps = static_cast<double>(telemetry_packets_) / elapsed_s;
    const double kbytes_per_s = pps * static_cast<double>(kAntiBaseDataBytes) / 1000.0;
    const double avg_gap_ms = telemetry_gaps_ > 0
        ? telemetry_gap_sum_ms_ / static_cast<double>(telemetry_gaps_)
        : 0.0;
    std::ostringstream oss;
    oss << "[M4 TX] pps=" << std::fixed << std::setprecision(2) << pps
        << " logical=" << kbytes_per_s << "kB/s"
        << " gap=" << telemetry_gap_min_ms_ << "/" << avg_gap_ms << "/" << telemetry_gap_max_ms_ << "ms"
        << " q=" << queued
        << " violation=" << telemetry_under_gap_;
    if (telemetry_under_gap_ > 0) {
        LOG(WARNING) << "[M4 TX LIMIT VIOLATION] " << oss.str();
    } else {
        LOG(INFO) << oss.str();
    }
}

}  // namespace antibase
