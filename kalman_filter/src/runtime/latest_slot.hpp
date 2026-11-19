#ifndef AUTOAIM_RUNTIME_LATEST_SLOT_HPP
#define AUTOAIM_RUNTIME_LATEST_SLOT_HPP

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stop_token>
#include <utility>

namespace realtime {

/*
 * 最新值槽：生产者覆盖，消费者只拿最新的一份。
 *
 * 适用场景：帧 -> 感知、状态 -> 控制、指令 -> 串口发送、调试包。
 * 特点：
 *   - 不做排队，天然丢弃旧值，延迟不随积压增长；
 *   - 每次 publish 序号 +1，消费者可以判断中间有没有漏掉更新；
 *   - T 需要可拷贝（copyLatest / tryTakeNewer 都会拷贝一份）。
 *     帧这类大对象请存 std::shared_ptr<const FramePacket>，拷贝只加引用计数。
 *
 * 生命周期：槽本身必须比所有使用者活得久；析构时还有线程在 waitNewer 属于未定义行为。
 */
template <typename T>
class LatestSlot {
 public:
  LatestSlot() = default;

  LatestSlot(const LatestSlot&) = delete;
  LatestSlot& operator=(const LatestSlot&) = delete;

  // 覆盖旧值并唤醒所有等待者，返回本次发布的序号（从 1 开始）。
  uint64_t publish(T value) {
    uint64_t published = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      value_ = std::move(value);
      published = ++seq_;
    }
    cv_.notify_all();
    return published;
  }

  // 读取最新值但不消费。槽里还没有值时返回 false。
  bool copyLatest(T& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!value_) return false;
    out = *value_;
    return true;
  }

  // 只在序号比 seen 新时取值，不阻塞。成功时把 seen 推进到当前序号。
  bool tryTakeNewer(uint64_t& seen, T& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return takeIfNewerUnlocked(seen, out, nullptr);
  }

  // 阻塞等待比 seen 新的值。stop 被请求时返回 false。
  bool waitNewer(uint64_t& seen, T& out, std::stop_token stop) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, stop, [&] { return seq_ > seen; });
    return takeIfNewerUnlocked(seen, out, &stop);
  }

  // 带超时的版本。超时、stop 或没有更新都返回 false。
  bool waitNewer(uint64_t& seen, T& out, std::stop_token stop,
                 std::chrono::steady_clock::duration timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    cv_.wait_until(lock, stop, deadline, [&] { return seq_ > seen; });
    return takeIfNewerUnlocked(seen, out, &stop);
  }

  bool hasValue() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_.has_value();
  }

  uint64_t seq() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return seq_;
  }

 private:
  // 调用方必须已持有 mutex_。stop 为空表示非阻塞调用，不检查停止标志。
  bool takeIfNewerUnlocked(uint64_t& seen, T& out, const std::stop_token* stop) const {
    if (stop != nullptr && stop->stop_requested()) return false;
    if (!value_ || seq_ <= seen) return false;
    out = *value_;
    seen = seq_;
    return true;
  }

  mutable std::mutex mutex_;
  std::condition_variable_any cv_;
  std::optional<T> value_;
  uint64_t seq_{};
};

}  // namespace realtime

#endif
