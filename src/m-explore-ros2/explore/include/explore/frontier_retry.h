#ifndef EXPLORE_FRONTIER_RETRY_H_
#define EXPLORE_FRONTIER_RETRY_H_
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace explore {
// Retry bookkeeping uses simulation seconds, so pausing does not consume
// retries.
class FrontierRetry {
  struct Entry {
    geometry_msgs::msg::Point point;
    double until;
    unsigned attempts;
  };
  std::vector<Entry> entries_;

 public:
  double radius = .25;
  double failure_cooldown = 20.;
  double success_cooldown = 6.;
  unsigned max_attempts = 3;
  bool blocked(const geometry_msgs::msg::Point& point, double now) const {
    for (const auto& entry : entries_) {
      if (std::hypot(point.x - entry.point.x, point.y - entry.point.y) <
              radius &&
          (entry.attempts >= max_attempts || now < entry.until))
        return true;
    }
    return false;
  }
  bool pending(double now) const {
    return std::any_of(entries_.begin(), entries_.end(),
                       [this, now](const Entry& e) {
                         return e.attempts < max_attempts && now < e.until;
                       });
  }
  void record(const geometry_msgs::msg::Point& point, double now,
              bool succeeded) {
    auto entry = std::find_if(
        entries_.begin(), entries_.end(), [this, &point](const Entry& e) {
          return std::hypot(point.x - e.point.x, point.y - e.point.y) < radius;
        });
    if (entry == entries_.end()) {
      entries_.push_back({point, now, 0});
      entry = entries_.end() - 1;
    }
    ++entry->attempts;
    entry->until = now + (succeeded ? success_cooldown : failure_cooldown);
  }
};
}  // namespace explore
#endif
