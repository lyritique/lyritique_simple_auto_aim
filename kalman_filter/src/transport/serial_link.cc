#include "transport/serial_link.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace comm {
namespace {

constexpr int kSendTimeoutMs = 100;

speed_t baudToSpeed(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return 0;
  }
}

}  // namespace

// 析构函数
SerialLink::~SerialLink() { close(); }

bool SerialLink::open(const char* dev, int baud) {
  close();

  if (dev == nullptr) return false;

  const speed_t speed = baudToSpeed(baud);
  if (speed == 0) return false;

  const int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return false;

  // 新建一个配置盒子
  termios options{};
  // 把当前串口通信模式写进配置盒子
  if (::tcgetattr(fd, &options) != 0) {
    ::close(fd);
    return false;
  }

  // 开始修改配置
  // 调为原始模式，让数据不会接受内核的修改
  ::cfmakeraw(&options);
  // 清掉旧的数据位、校验位、停止位设置
  options.c_cflag &= ~static_cast<tcflag_t>(CSIZE | PARENB | CSTOPB);
  // 数值位为8,忽略调制解调器,可以读也可以输入
  options.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
  // 关闭硬件流控
#ifdef CRTSCTS
  options.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
#endif
  // 让read的不等待任何字节，不等待任何时间
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (::cfsetispeed(&options, speed) != 0 ||                                           // 设置输入输出波特率
      ::cfsetospeed(&options, speed) != 0 || ::tcsetattr(fd, TCSANOW, &options) != 0)  // 把option写回内核
  {
    ::close(fd);
    return false;
  }

  // 清空缓存区
  ::tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  rx_size_ = 0;
  return true;
}

bool SerialLink::send(const VisionToGimbal& packet) {
  if (fd_ < 0) return false;

  std::array<uint8_t, kFrameSize> frame{};
  std::memcpy(frame.data(), &packet, sizeof(packet));
  frame[0] = 0x5A;
  frame[1] = 0x01;
  packVtoQ(frame.data());

  std::size_t sent = 0;
  while (sent < frame.size()) {
    const ssize_t n = ::write(fd_, frame.data() + sent, frame.size() - sent);

    if (n > 0) {
      sent += static_cast<std::size_t>(n);
      continue;
    }

    if (n < 0 && errno == EINTR) continue;

    // 内核缓冲区满了
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      // 构造一个 pollfd 结构,相当于 poll 的配置
      pollfd pfd{};
      pfd.fd = fd_;
      // poll关心的事件 可写
      pfd.events = POLLOUT;

      int ret{};
      do {
        ret = ::poll(&pfd, 1, kSendTimeoutMs);
      } while (ret < 0 && errno == EINTR);

      // 如果可写，就回去重试
      if (ret > 0 && (pfd.revents & POLLOUT)) continue;
    }

    return false;
  }

  return true;
}

bool SerialLink::poll(GimbalToVision& out) {
  if (fd_ < 0) return false;

  // 将rx_buf缓存逐个得出out
  auto extractFrame = [&]() -> bool {
    for (;;) {
      if (rx_size_ < 2) return false;

      const auto begin = rx_buffer_.begin();
      const auto end = begin + static_cast<std::ptrdiff_t>(rx_size_);
      const std::array<uint8_t, 2> header{0x5A, 0x01};
      const auto found = std::search(begin, end, header.begin(), header.end());

      if (found == end) {
        // 没找到帧头时只保留可能是帧头前半部分的最后一个字节。
        const bool keep_magic = rx_buffer_[rx_size_ - 1] == header[0];
        rx_buffer_[0] = header[0];
        rx_size_ = keep_magic ? 1 : 0;
        return false;
      }

      const auto offset = static_cast<std::size_t>(found - begin);
      if (offset > 0) {
        std::memmove(rx_buffer_.data(), rx_buffer_.data() + offset, rx_size_ - offset);
        rx_size_ -= offset;
      }

      if (rx_size_ < kFrameSize) return false;

      GimbalToVision parsed{};
      if (parse(rx_buffer_.data(), parsed)) {
        out = parsed;
        const std::size_t remain = rx_size_ - kFrameSize;
        if (remain > 0) {
          std::memmove(rx_buffer_.data(), rx_buffer_.data() + kFrameSize, remain);
        }
        rx_size_ = remain;
        return true;
      }

      // 这是一个假帧头：丢掉一个字节，继续向后搜索。
      std::memmove(rx_buffer_.data(), rx_buffer_.data() + 1, rx_size_ - 1);
      --rx_size_;
    }
  };

  if (extractFrame()) return true;

  std::array<uint8_t, 256> incoming{};
  for (;;) {
    const std::size_t space = rx_buffer_.size() - rx_size_;
    if (space == 0) return false;  // rx_buf缓存满了

    // 将内核缓冲输入incoming缓存
    const ssize_t n = ::read(fd_, incoming.data(), std::min(space, incoming.size()));

    if (n > 0) {
      // 有效的存进rx_buf
      std::memcpy(rx_buffer_.data() + rx_size_, incoming.data(), static_cast<std::size_t>(n));
      rx_size_ += static_cast<std::size_t>(n);

      if (extractFrame()) return true;
      continue;
    }

    if (n == 0) return false;
    if (errno == EINTR) continue;  // 信号打断
    if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
    return false;
  }
}

void SerialLink::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }

  rx_size_ = 0;
  rx_buffer_.fill(0);
}

}  // namespace comm
