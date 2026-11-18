#ifndef AUTOAIM_SENSORS_CAMERA_SOURCE_HPP
#define AUTOAIM_SENSORS_CAMERA_SOURCE_HPP

#include <string>

#include "core/frame.hpp"

namespace camera {

struct SourceInfo {
  int width{};
  int height{};
  double frame_rate{};
  uint64_t frame_count{};
};

class CameraSource {
 public:
  virtual ~CameraSource() = default;

  virtual bool open(std::string* error = nullptr) = 0;
  virtual core::FrameReadStatus read(core::TimestampedFrame& frame, std::string* error = nullptr) = 0;
  virtual void close() = 0;

  virtual bool isOpen() const = 0;
  virtual SourceInfo info() const = 0;
};

}  // namespace camera

#endif
