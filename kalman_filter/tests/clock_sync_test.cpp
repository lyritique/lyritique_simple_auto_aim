#include "timing/clock_sync.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

bool near(double lhs, double rhs, double tol = 1e-6) { return std::abs(lhs - rhs) <= tol; }

}  // namespace

int main() {
  kalfilter::ClockSync clock(0.002);
  uint64_t board_time = 0;
  double host_time = 0.0;

  // 第一关：DWT 戳是毫秒，必须按 1000 倍展开成微秒，否则整条时间轴会慢 1000 倍。
  if (!clock.observeAndMap(267500.0F, 1000.0, board_time, host_time) || board_time != 267500000ULL ||
      !near(host_time, 999.998)) {
    std::cerr << "millisecond to microsecond scale failed: board=" << board_time << " host=" << host_time << "\n";
    return 1;
  }

  // 第二关：相邻 1 ms 的两个样本，上位机时间也必须前进 1 ms，不能被压成 1 μs。
  const double first_host_time = host_time;
  if (!clock.observeAndMap(267501.0F, 1000.001, board_time, host_time) || board_time != 267501000ULL ||
      !near(host_time - first_host_time, 0.001)) {
    std::cerr << "1 ms step mapping failed: delta=" << (host_time - first_host_time) << "\n";
    return 2;
  }

  // 第三关：32 位毫秒计数回绕，补的是 2^32 ms 而不是 2^32 μs。
  kalfilter::ClockSync wrap_clock(0.002);
  // 2^32 - 512，float 能精确表示的边界值之一。
  if (!wrap_clock.observeAndMap(4294966784.0F, 2000.0, board_time, host_time)) {
    std::cerr << "pre-wrap sample rejected\n";
    return 3;
  }
  const double before_wrap = host_time;

  if (!wrap_clock.observeAndMap(4.0F, 2000.516, board_time, host_time) || board_time != 4294967300000ULL ||
      !near(host_time - before_wrap, 0.516)) {
    std::cerr << "wrap mapping failed: board=" << board_time << " delta=" << (host_time - before_wrap) << "\n";
    return 4;
  }

  // 第四关：乱序和重复的戳都要被挡在门外。
  if (clock.observeAndMap(267500.0F, 1000.002, board_time, host_time)) {
    std::cerr << "out-of-order timestamp accepted\n";
    return 5;
  }
  if (clock.observeAndMap(267501.0F, 1000.003, board_time, host_time)) {
    std::cerr << "duplicate timestamp accepted\n";
    return 6;
  }

  std::cout << "clock sync test passed\n";
  return 0;
}
