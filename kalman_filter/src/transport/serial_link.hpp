#ifndef AUTOAIM_SERIAL_LINK_HPP
#define AUTOAIM_SERIAL_LINK_HPP

#include <array>
#include <cstddef>
#include <string>

#include "transport/protocol.hpp"

namespace comm {

class SerialLink {
 public:
  SerialLink() = default;
  ~SerialLink();

  SerialLink(const SerialLink&) = delete;
  SerialLink& operator=(const SerialLink&) = delete;

  // 打开串口。失败原因放在 lastError() 里，包含 errno 文案和现有的候选设备。
  bool open(const char* dev, int baud);
  const std::string& lastError() const noexcept { return last_error_; }
  bool send(const VisionToGimbal& packet);
  bool poll(GimbalToVision& out);
  void close();

 private:
  std::string last_error_{};
  int fd_ = -1;
  std::array<uint8_t, 128> rx_buffer_{};
  std::size_t rx_size_{};
};

}  // namespace comm

#endif
