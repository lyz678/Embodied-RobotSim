#pragma once
#include <cmath>
namespace explore {
// Measure cumulative progress from an anchor, not tiny per-tick displacement.
// Time is simulation time: render FPS and pause duration do not change
// decisions.
class ProgressWatchdog {
 public:
  double distance = .05, angle = .17, timeout = 20.;
  void reset() { initialized_ = false; }
  bool stalled(double x, double y, double yaw, double now) {
    const auto changed =
        std::abs(std::atan2(std::sin(yaw - yaw_), std::cos(yaw - yaw_)));
    if (!initialized_ || now < checked_at_ ||
        std::hypot(x - x_, y - y_) >= distance || changed >= angle) {
      x_ = x;
      y_ = y;
      yaw_ = yaw;
      progress_at_ = now;
      initialized_ = true;
    }
    checked_at_ = now;
    return now - progress_at_ >= timeout;
  }

 private:
  bool initialized_ = false;
  double x_ = 0., y_ = 0., yaw_ = 0., progress_at_ = 0., checked_at_ = 0.;
};
}  // namespace explore
