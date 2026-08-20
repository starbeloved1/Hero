#include "AntiBase/include/Packetizer0310.hpp"

#include <algorithm>
#include <cstring>

namespace antibase {

Packetizer0310::Packetizer0310(std::size_t max_packet_bytes)
    : max_packet_bytes_(max_packet_bytes < kHeaderBytes ? kHeaderBytes : max_packet_bytes) {}

std::size_t Packetizer0310::packetSize() const {
    return max_packet_bytes_;
}

std::size_t Packetizer0310::payloadSize() const {
    return max_packet_bytes_ - kHeaderBytes;
}

AntiBasePacket Packetizer0310::makePacket(
    const uint8_t* raw,
    std::size_t len,
    uint64_t sequence_id
) const {
    AntiBasePacket packet;
    packet.sequence_id = sequence_id;
    packet.data.assign(payloadSize(), 0);

    std::size_t copy_len = std::min(len, payloadSize());
    if (raw != nullptr && copy_len > 0) {
        std::memcpy(packet.data.data(), raw, copy_len);
    }

    return packet;
}

}  // namespace antibase
