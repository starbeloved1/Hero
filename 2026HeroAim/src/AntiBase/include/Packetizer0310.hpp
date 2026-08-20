#ifndef HERO_26_ANTIBASE_PACKETIZER0310_HPP
#define HERO_26_ANTIBASE_PACKETIZER0310_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace antibase {

struct AntiBasePacket {
    uint64_t sequence_id = 0;
    std::vector<uint8_t> data;
};

class Packetizer0310 {
public:
    // 300B 逻辑包只携带单调序号；时间戳不参与重组、限速或解码，省出 8B 给 H.264。
    static constexpr std::size_t kHeaderBytes = sizeof(uint64_t);

    explicit Packetizer0310(std::size_t max_packet_bytes);

    std::size_t packetSize() const;
    std::size_t payloadSize() const;
    AntiBasePacket makePacket(
        const uint8_t* raw,
        std::size_t len,
        uint64_t sequence_id
    ) const;

private:
    std::size_t max_packet_bytes_;
};

}  // namespace antibase

#endif
