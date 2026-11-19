#ifndef AUTOAIM_RUNTIME_BOUNDED_QUEUE_HPP
#define AUTOAIM_RUNTIME_BOUNDED_QUEUE_HPP

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <stop_token>
#include <utility>

namespace realtime {

/*
 * 有界队列：容量满了丢最旧，保证延迟不会随积压增长。
 *
 * 适用场景：必须保序、不能合并的数据。首版流水线尽量用 LatestSlot；
 * 只有多推理实例这种“结果必须按帧序重排”的场合才用队列。
 *
 * 语义：
 *   - push 在关闭后返回 false，值不入队；
 *   - 队列满时先丢队头再入队，droppedCount() 记录丢了多少；
 *   - close 之后不再接收新值，pop 会把剩余元素取完再返回 false；
 *   - pop 支持 stop_token，线程池/流水线退出时不会卡死。
 */
template <typename T>
class BoundedQueue {
 public:
  explicit BoundedQueue(std::size_t capacity) : capacity_(capacity == 0 ? 1 : capacity) {}

  BoundedQueue(const BoundedQueue&) = delete;
  BoundedQueue& operator=(const BoundedQueue&) = delete;

  // 入队；队列已关闭时返回 false。
  bool push(T value) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) return false;
      while (queue_.size() >= capacity_) {
        queue_.pop_front();
        ++dropped_;
      }
      queue_.push_back(std::move(value));
    }
    cv_.notify_one();
    return true;
  }

  // 阻塞等一个元素；stop 被请求或关闭且已取空时返回 false。
  bool pop(T& out, std::stop_token stop) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, stop, [this] { return !queue_.empty() || closed_; });
    return takeUnlocked(out, stop);
  }

  // 带超时的 pop；超时同样返回 false。
  bool pop(T& out, std::stop_token stop, std::chrono::steady_clock::duration timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    cv_.wait_until(lock, stop, deadline, [this] { return !queue_.empty() || closed_; });
    return takeUnlocked(out, stop);
  }

  // 非阻塞取队头。
  bool tryPop(T& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return false;
    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  // 关闭队列：之后 push 失败，pop 把剩余元素取完后返回 false。
  void close() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    cv_.notify_all();
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  bool closed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

  std::size_t capacity() const noexcept { return capacity_; }

  uint64_t droppedCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_;
  }

 private:
  // 调用方必须已持有 mutex_。
  bool takeUnlocked(T& out, const std::stop_token& stop) {
    if (stop.stop_requested()) return false;
    if (queue_.empty()) return false;  // 只可能是 closed_ 且已取空
    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  const std::size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable_any cv_;
  std::deque<T> queue_;
  uint64_t dropped_{};
  bool closed_{};
};

}  // namespace realtime

#endif
