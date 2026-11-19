#include "runtime/bounded_queue.hpp"

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

  realtime::BoundedQueue<int> queue(2);
  int value = -1;

  check(queue.capacity() == 2, "容量应为 2");
  check(!queue.tryPop(value), "空队列 tryPop 应返回 false");
  check(queue.push(1) && queue.push(2), "前两次 push 应成功");
  check(queue.push(3), "第三次 push 应成功并丢最旧");
  check(queue.droppedCount() == 1, "应记录一次 DropOldest");
  check(queue.size() == 2, "队列长度应保持为容量");

  check(queue.pop(value, std::stop_token{}) && value == 2, "DropOldest 后队头应为 2");
  check(queue.pop(value, std::stop_token{}) && value == 3, "第二个应为 3");
  check(!queue.tryPop(value), "取空后 tryPop 应返回 false");

  // 阻塞等待：push 后应被唤醒。
  realtime::BoundedQueue<int> blocking(1);
  std::atomic<int> received{-1};
  std::jthread consumer([&](std::stop_token stop) {
    int item = 0;
    if (blocking.pop(item, stop)) {
      received.store(item);
    }
  });
  std::this_thread::sleep_for(20ms);
  blocking.push(7);
  consumer.join();
  check(received.load() == 7, "pop 应被 push 唤醒并拿到 7");

  // 超时：没有数据时按配置返回 false。
  realtime::BoundedQueue<int> timeout_queue(1);
  check(!timeout_queue.pop(value, std::stop_token{}, 10ms), "空队列 pop 应超时");

  // 关闭：push 失败，剩余元素可以取完，取空后 pop 返回 false。
  realtime::BoundedQueue<int> closing(2);
  closing.push(9);
  closing.close();
  check(closing.closed(), "close 后 closed 应为 true");
  check(!closing.push(10), "close 后 push 应失败");
  check(closing.pop(value, std::stop_token{}) && value == 9, "close 后应先取完剩余元素");
  check(!closing.pop(value, std::stop_token{}), "取空后 pop 应返回 false");

  // 停止等待：request_stop 必须能唤醒阻塞的 pop。
  realtime::BoundedQueue<int> idle(1);
  std::atomic<bool> exited{false};
  std::jthread waiter([&](std::stop_token stop) {
    int item = 0;
    (void)idle.pop(item, stop);
    exited.store(true);
  });
  std::this_thread::sleep_for(20ms);
  waiter.request_stop();
  waiter.join();
  check(exited.load(), "request_stop 应唤醒阻塞的 pop");

  if (failures != 0) {
    std::cerr << failures << " 项失败\n";
    return 1;
  }
  std::cout << "bounded queue test passed\n";
  return 0;
}
