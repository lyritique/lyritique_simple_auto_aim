#include "runtime/latest_slot.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::cerr << "FAIL: " << what << '\n';
    ++failures;
  }
}

}  // namespace

int main() {
  using namespace std::chrono_literals;

  realtime::LatestSlot<int> slot;
  int value = -1;

  check(!slot.copyLatest(value), "空槽 copyLatest 应返回 false");
  check(slot.publish(1) == 1, "首次 publish 序号应为 1");
  check(slot.copyLatest(value) && value == 1, "copyLatest 应拿到 1");
  check(slot.publish(2) == 2, "第二次 publish 序号应为 2");
  check(slot.seq() == 2, "seq 应推进到 2");

  uint64_t seen = 0;
  check(slot.tryTakeNewer(seen, value) && value == 2 && seen == 2, "tryTakeNewer 应拿到最新值 2");
  check(!slot.tryTakeNewer(seen, value), "没有更新时 tryTakeNewer 应返回 false");

  // 连续发布三个值，消费者只应看到最后一个，序号一次推进到 3。
  slot.publish(3);
  slot.publish(4);
  check(slot.publish(5) == 5, "连续 publish 序号应连续递增");
  check(slot.tryTakeNewer(seen, value) && value == 5 && seen == 5, "应合并到最新值 5");

  const auto timeout_start = std::chrono::steady_clock::now();
  check(!slot.waitNewer(seen, value, std::stop_token{}, 10ms), "没有更新时 waitNewer 应超时");
  check(std::chrono::steady_clock::now() - timeout_start >= 5ms, "waitNewer 应真的等过一段时间");

  // 阻塞等待：发布后应被唤醒，并拿到新值。
  realtime::LatestSlot<int> wake_slot;
  std::atomic<int> received{-1};
  std::jthread consumer([&](std::stop_token stop) {
    uint64_t consumer_seen = 0;
    int consumer_value = 0;
    if (wake_slot.waitNewer(consumer_seen, consumer_value, stop)) {
      received.store(consumer_value);
    }
  });
  std::this_thread::sleep_for(20ms);
  wake_slot.publish(42);
  consumer.join();
  check(received.load() == 42, "waitNewer 应被 publish 唤醒并拿到 42");

  // 停止等待：没有新值时 request_stop 必须能唤醒消费者。
  realtime::LatestSlot<int> idle_slot;
  std::atomic<bool> exited{false};
  std::jthread waiter([&](std::stop_token stop) {
    uint64_t waiter_seen = 0;
    int waiter_value = 0;
    (void)idle_slot.waitNewer(waiter_seen, waiter_value, stop);
    exited.store(true);
  });
  std::this_thread::sleep_for(20ms);
  waiter.request_stop();
  waiter.join();
  check(exited.load(), "request_stop 应唤醒 waitNewer");

  if (failures != 0) {
    std::cerr << failures << " 项失败\n";
    return 1;
  }
  std::cout << "latest slot test passed\n";
  return 0;
}
