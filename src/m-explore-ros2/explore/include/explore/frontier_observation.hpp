#ifndef EXPLORE_FRONTIER_OBSERVATION_HPP_
#define EXPLORE_FRONTIER_OBSERVATION_HPP_

#include <cmath>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace explore {
// Check the original UNKNOWN frontier cells, never the already-free goal or
// inflated navigation costs. World coordinates survive map resizing.
inline bool frontierObserved(
    const nav_msgs::msg::OccupancyGrid& map,
    const std::vector<geometry_msgs::msg::Point>& points) {
  const auto& info = map.info;
  if (points.empty() || !std::isfinite(info.resolution) || info.resolution <= 0 ||
      map.data.size() != static_cast<size_t>(info.width) * info.height)
    return false;
  const auto& q = info.origin.orientation;
  const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z));
  for (const auto& point : points) {
    const double dx = point.x - info.origin.position.x;
    const double dy = point.y - info.origin.position.y;
    const double x = (std::cos(yaw) * dx + std::sin(yaw) * dy) / info.resolution;
    const double y = (-std::sin(yaw) * dx + std::cos(yaw) * dy) / info.resolution;
    if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 ||
        x >= info.width || y >= info.height)
      return false;
    if (map.data[static_cast<size_t>(std::floor(y)) * info.width +
                 static_cast<size_t>(std::floor(x))] < 0)
      return false;
  }
  return true;
}
}  // namespace explore
#endif
