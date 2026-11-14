#ifndef AUTOAIM_SERIAL_LINK_HPP
#define AUTOAIM_SERIAL_LINK_HPP

#include <array>
#include <cstddef>

#include "transport/protocol.hpp"

namespace comm {

class SerialLink {
 public:
  SerialLink() = default;
  ~SerialLink();

  SerialLink(const SerialLink&) = delete;
  SerialLink& operator=(const SerialLink&) = delete;

  bool open(const char* dev, int baud);
  bool send(const VisionToGimbal& packet);
  bool poll(GimbalToVision& out);
  void close();

 private:
  int fd_ = -1;
  std::array<uint8_t, 128> rx_buffer_{};
  std::size_t rx_size_{};
};

}  // namespace comm

#endif
