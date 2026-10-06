#include <explore/costmap_tools.h>
#include <explore/frontier_search.h>

#include <geometry_msgs/msg/point.hpp>
#include <mutex>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_exploration
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size, rclcpp::Logger logger,
                               unsigned char max_travel_cost)
  : costmap_(costmap)
  , max_travel_cost_(max_travel_cost)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
  , logger_(logger)
{
}

std::vector<Frontier>
FrontierSearch::searchFrom(geometry_msgs::msg::Point position)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    RCLCPP_ERROR(logger_, "[FrontierSearch] Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  reachable_.assign(size_x_ * size_y_, false);
  std::queue<unsigned int> bfs;
  const unsigned int pos = costmap_->getIndex(mx, my);
  unsigned int start = pos;
  if (map_[start] > max_travel_cost_ &&
      !nearestCell(start, pos, FREE_SPACE, *costmap_)) {
    RCLCPP_WARN(logger_, "No traversable cell near robot; retry after costmap update");
    return frontier_list;
  }
  bfs.push(start);
  reachable_[start] = true;
  // Inflation gradients are traversable in both directions. The old descending
  // flood-fill could not cross from clear space into inflated corner cells.
  while (!bfs.empty()) {
    const auto idx = bfs.front();
    bfs.pop();
    for (auto nbr : nhood4(idx, *costmap_)) {
      if (!reachable_[nbr] && map_[nbr] <= max_travel_cost_) {
        reachable_[nbr] = true;
        bfs.push(nbr);
      }
    }
  }
  // Only consider unknown boundaries adjacent to the robot's reachable component.
  for (unsigned int idx = 0; idx < reachable_.size(); ++idx) {
    if (!reachable_[idx]) continue;
    for (auto nbr : nhood4(idx, *costmap_)) {
      if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        auto frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (frontier.size * costmap_->getResolution() >= min_frontier_size_) {
          frontier_list.push_back(frontier);
        }
      }
    }
  }

  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  Frontier output;
  output.size = 0;
  output.centroid.x = output.centroid.y = 0.;
  output.min_distance = std::numeric_limits<double>::infinity();
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);
  while (!bfs.empty()) {
    const auto idx = bfs.front();
    bfs.pop();
    unsigned int mx, my;
    geometry_msgs::msg::Point point;
    costmap_->indexToCells(idx, mx, my);
    costmap_->mapToWorld(mx, my, point.x, point.y);
    output.points.push_back(point);
    output.centroid.x += point.x;
    output.centroid.y += point.y;
    ++output.size;
    for (auto nbr : nhood8(idx, *costmap_)) {
      if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        bfs.push(nbr);
      }
    }
  }
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  // A centroid can lie in a wall, or inside unknown space. Pick a known,
  // reachable neighbor closest to it, facing the unknown boundary on arrival.
  double best = std::numeric_limits<double>::infinity();
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);
  for (const auto &point : output.points) {
    unsigned int mx, my;
    costmap_->worldToMap(point.x, point.y, mx, my);
    for (auto nbr : nhood4(costmap_->getIndex(mx, my), *costmap_)) {
      if (!reachable_[nbr]) continue;
      geometry_msgs::msg::Point candidate;
      costmap_->indexToCells(nbr, mx, my);
      costmap_->mapToWorld(mx, my, candidate.x, candidate.y);
      output.min_distance = std::min(output.min_distance,
          std::hypot(candidate.x-reference_x, candidate.y-reference_y));
      const auto score = std::hypot(candidate.x-output.centroid.x,
                                   candidate.y-output.centroid.y);
      if (score < best) {
        best = score;
        output.middle = candidate;
      }
    }
  }
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (reachable_[nbr]) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}
}  // namespace frontier_exploration
