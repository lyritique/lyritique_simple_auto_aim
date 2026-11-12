# include "transport/protocol.hpp"

# include <cstring>

namespace comm
{

bool checkQtoV(const uint8_t* p)
{
    if (p == nullptr) return false;

    uint16_t sum {};
    for (int i {}; i < 62; ++i) sum += p[i];

    const uint16_t received =
        static_cast<uint16_t>(p[62]) |
        (static_cast<uint16_t>(p[63]) << 8);
    return sum == received;
}

void packVtoQ(uint8_t* p)
{
    if (p == nullptr) return;

    uint16_t sum {};
    for (int i {}; i < 61; ++i) sum += p[i];

    p[61] = static_cast<uint8_t>(sum & 0xFF);
    p[62] = static_cast<uint8_t>((sum >> 8) & 0xFF);
    p[63] = 0x55;
}

bool parse(const uint8_t* frame, GimbalToVision& out)
{
    if (frame == nullptr) return false;
    if (frame[0] != 0x5A || frame[1] != 0x01) return false;
    if (frame[61] != 0x55) return false;
    if (!checkQtoV(frame)) return false;

    std::memcpy(&out, frame, sizeof(GimbalToVision));
    return true;
}

}   // namespace comm
