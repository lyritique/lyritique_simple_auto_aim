# include "transport/protocol.hpp"
# include "transport/serial_link.hpp"

# include <array>
# include <cerrno>
# include <cmath>
# include <cstring>
# include <iostream>
# include <pty.h>
# include <poll.h>
# include <unistd.h>

namespace
{

bool writeAll(int fd, const uint8_t* data, std::size_t size)
{
    std::size_t written = 0;
    while (written < size)
    {
        const ssize_t count = ::write(
            fd, data + written, size - written);
        if (count < 0)
        {
            if (errno == EINTR) continue;
            return false;
        }
        written += static_cast<std::size_t>(count);
    }
    return true;
}

std::array<uint8_t, comm::kFrameSize> makeIncoming(float yaw)
{
    comm::GimbalToVision packet {};
    packet.head[0] = 0x5A;
    packet.head[1] = 0x01;
    packet.DWT_stamp = 1234.0F;
    packet.q[0] = 1.0F;
    packet.yaw = yaw;
    packet.tail = 0x55;

    std::array<uint8_t, comm::kFrameSize> frame {};
    std::memcpy(frame.data(), &packet, sizeof(packet));

    uint16_t sum = 0;
    for (std::size_t i = 0; i < 62; ++i) sum += frame[i];
    frame[62] = static_cast<uint8_t>(sum);
    frame[63] = static_cast<uint8_t>(sum >> 8);
    return frame;
}

}   // namespace

int main()
{
    int master = -1;
    int slave = -1;
    char slave_name[128] {};
    if (::openpty(&master, &slave, slave_name, nullptr, nullptr) != 0)
    {
        std::perror("openpty");
        return 1;
    }

    comm::SerialLink link;
    if (!link.open(slave_name, 115200))
    {
        std::cerr << "serial open failed\n";
        return 2;
    }
    ::close(slave);

    const auto first = makeIncoming(1.25F);
    const uint8_t noise[] = {0x10, 0x11, 0x22};
    if (!writeAll(master, noise, sizeof(noise)) ||
        !writeAll(master, first.data(), 17))
    {
        std::cerr << "write prefix failed\n";
        return 3;
    }

    comm::GimbalToVision received {};
    if (link.poll(received))
    {
        std::cerr << "accepted an incomplete frame\n";
        return 4;
    }

    if (!writeAll(master, first.data() + 17, first.size() - 17) ||
        !link.poll(received) ||
        std::abs(received.yaw - 1.25F) > 1e-6F)
    {
        std::cerr << "incoming frame parse failed\n";
        return 5;
    }

    comm::VisionToGimbal outgoing {};
    outgoing.time_stamp = 42;
    outgoing.mode = 2;
    if (!link.send(outgoing))
    {
        std::cerr << "serial send failed\n";
        return 6;
    }

    pollfd pfd {};
    pfd.fd = master;
    pfd.events = POLLIN;
    if (::poll(&pfd, 1, 1000) <= 0)
    {
        std::cerr << "outgoing frame timeout\n";
        return 7;
    }

    std::array<uint8_t, comm::kFrameSize> wire {};
    std::size_t size = 0;
    while (size < wire.size())
    {
        const ssize_t count = ::read(
            master, wire.data() + size, wire.size() - size);
        if (count <= 0) return 8;
        size += static_cast<std::size_t>(count);
    }

    uint16_t sum = 0;
    for (std::size_t i = 0; i < 61; ++i) sum += wire[i];
    const uint16_t received_sum =
        static_cast<uint16_t>(wire[61]) |
        (static_cast<uint16_t>(wire[62]) << 8);

    const bool valid =
        wire[0] == 0x5A &&
        wire[1] == 0x01 &&
        wire[63] == 0x55 &&
        sum == received_sum;

    if (!valid)
    {
        std::cerr << "outgoing frame layout failed\n";
        return 9;
    }

    std::cout << "protocol and serial test passed\n";
    return 0;
}
