#include <explore/frontier_observation.hpp>
#include <explore/frontier_retry.h>
#include <explore/frontier_search.h>
#include <gtest/gtest.h>

#include <nav2_costmap_2d/cost_values.hpp>

using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

TEST(FrontierCoverage, TraversesInflationToFindCorners) {
  Costmap2D map(9, 7, .1, 0, 0, NO_INFORMATION);
  for (unsigned x = 1; x <= 6; ++x)
    for (unsigned y = 1; y <= 5; ++y) map.setCost(x, y, 150);
  map.setCost(2, 3, FREE_SPACE);
  geometry_msgs::msg::Point robot;
  map.mapToWorld(2, 3, robot.x, robot.y);
  frontier_exploration::FrontierSearch search(&map, 5, .5, .1,
                                              rclcpp::get_logger("test"));
  const auto frontiers = search.searchFrom(robot);
  ASSERT_FALSE(frontiers.empty());
  bool far_corner = false;
  for (const auto& frontier : frontiers) {
    unsigned x, y;
    ASSERT_TRUE(map.worldToMap(frontier.middle.x, frontier.middle.y, x, y));
    EXPECT_LE(map.getCost(x, y), 200);
    for (const auto& point : frontier.points)
      if (point.x > .69 && point.y > .49) far_corner = true;
  }
  EXPECT_TRUE(far_corner);
}

TEST(FrontierCoverage, KeepsSingleCellAndItsUnbiasedCenter) {
  Costmap2D map(5, 5, .1, 10, 20, LETHAL_OBSTACLE);
  map.setCost(1, 2, FREE_SPACE);
  map.setCost(2, 2, FREE_SPACE);
  map.setCost(3, 2, NO_INFORMATION);
  geometry_msgs::msg::Point robot;
  map.mapToWorld(1, 2, robot.x, robot.y);
  frontier_exploration::FrontierSearch search(&map, 5, 0, .1,
                                              rclcpp::get_logger("test"));
  const auto frontiers = search.searchFrom(robot);
  ASSERT_EQ(frontiers.size(), 1u);
  const auto& f = frontiers.front();
  EXPECT_EQ(f.size, 1u);
  ASSERT_EQ(f.points.size(), 1u);
  EXPECT_NEAR(f.centroid.x, 10.35, 1e-9);
  EXPECT_NEAR(f.centroid.y, 20.25, 1e-9);
  EXPECT_NEAR(f.middle.x, 10.25, 1e-9);
  // min_distance is already in meters, not cell units.
  EXPECT_NEAR(f.cost, .5, 1e-9);
}

TEST(FrontierCoverage, DoesNotCrossWallIntoDisconnectedSpace) {
  Costmap2D map(10, 5, .1, 0, 0, LETHAL_OBSTACLE);
  for (unsigned x = 1; x <= 3; ++x) map.setCost(x, 2, FREE_SPACE);
  for (unsigned x = 6; x <= 8; ++x) map.setCost(x, 2, FREE_SPACE);
  map.setCost(9, 2, NO_INFORMATION);
  geometry_msgs::msg::Point robot;
  map.mapToWorld(2, 2, robot.x, robot.y);
  frontier_exploration::FrontierSearch search(&map, 5, .5, .1,
                                              rclcpp::get_logger("test"));
  EXPECT_TRUE(search.searchFrom(robot).empty());
}

TEST(FrontierCoverage, DoesNotTraverseInscribedCosts) {
  Costmap2D map(6, 3, .1, 0, 0, LETHAL_OBSTACLE);
  map.setCost(1, 1, FREE_SPACE);
  map.setCost(2, 1, 253);
  map.setCost(3, 1, FREE_SPACE);
  map.setCost(4, 1, NO_INFORMATION);
  geometry_msgs::msg::Point robot;
  map.mapToWorld(1, 1, robot.x, robot.y);
  frontier_exploration::FrontierSearch search(&map, 5, .5, .1,
                                              rclcpp::get_logger("test"));
  EXPECT_TRUE(search.searchFrom(robot).empty());
}

TEST(FrontierRetry, DelaysRetriesAndBoundsPersistentFailures) {
  explore::FrontierRetry retry;
  geometry_msgs::msg::Point point;
  point.x = 1;
  point.y = 2;
  retry.record(point, 0, false);
  EXPECT_TRUE(retry.blocked(point, 19));
  EXPECT_TRUE(retry.pending(19));
  EXPECT_FALSE(retry.blocked(point, 20));
  EXPECT_FALSE(retry.pending(20));
  retry.record(point, 20, false);
  retry.record(point, 40, false);
  EXPECT_TRUE(retry.blocked(point, 1000));
  EXPECT_FALSE(retry.pending(1000));
  point.x += .5;
  EXPECT_FALSE(retry.blocked(point, 1000));
}

TEST(FrontierRetry, SuccessfulObservationCanBeRevisited) {
  explore::FrontierRetry retry;
  geometry_msgs::msg::Point point;
  retry.record(point, 10, true);
  EXPECT_TRUE(retry.blocked(point, 15));
  EXPECT_FALSE(retry.blocked(point, 16));
}

#include "explore/progress_watchdog.hpp"
TEST(ProgressWatchdog, SlowCumulativeMotionIsNotStuck) {
  explore::ProgressWatchdog progress;
  for(int i=0;i<100;++i) EXPECT_FALSE(progress.stalled(.034*i,0.,0.,2.*i));
}
TEST(ProgressWatchdog, PauseRewindAndStationaryTimeout) {
  explore::ProgressWatchdog progress;
  EXPECT_FALSE(progress.stalled(0.,0.,0.,10.));
  EXPECT_FALSE(progress.stalled(0.,0.,0.,10.)); // paused clock
  EXPECT_FALSE(progress.stalled(0.,0.,0.,29.));
  EXPECT_TRUE(progress.stalled(0.,0.,0.,30.));
  EXPECT_FALSE(progress.stalled(0.,0.,0.,1.)); // rewind resets
  EXPECT_FALSE(progress.stalled(0.,0.,.2,20.)); // turn is progress
  EXPECT_FALSE(progress.stalled(0.,0.,.2,39.));
}


TEST(FrontierObservation, ChecksUnknownBoundaryNotFreeNavigationGoal) {
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 5;
  map.info.height = 1;
  map.info.resolution = 1.;
  map.info.origin.orientation.w = 1.;
  map.data = {0, 0, -1, -1, 100};
  geometry_msgs::msg::Point boundary;
  boundary.x = 2.5;
  boundary.y = .5;
  const std::vector<geometry_msgs::msg::Point> points{boundary};
  EXPECT_FALSE(explore::frontierObserved(map, points));
  map.data[2] = 100;  // Observed obstacle counts, no need to reach it.
  EXPECT_TRUE(explore::frontierObserved(map, points));
  boundary.x = 3.5;
  EXPECT_FALSE(explore::frontierObserved(map, {points.front(), boundary}));
  map.data[3] = 0;
  EXPECT_TRUE(explore::frontierObserved(map, {points.front(), boundary}));
}

TEST(FrontierObservation, HandlesResizedMapAndMissingDataConservatively) {
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 3;
  map.info.height = 1;
  map.info.resolution = 1.;
  map.info.origin.position.x = 10.;
  map.info.origin.position.y = 20.;
  map.info.origin.orientation.w = 1.;
  map.data = {0, -1, 100};
  geometry_msgs::msg::Point boundary;
  boundary.x = 11.5;
  boundary.y = 20.5;
  EXPECT_FALSE(explore::frontierObserved(map, {boundary}));
  map.info.width = 4;
  map.info.origin.position.x = 9.;
  map.data = {-1, 0, 0, 100};
  EXPECT_TRUE(explore::frontierObserved(map, {boundary}));
  map.info.origin.position.x = 20.;
  EXPECT_FALSE(explore::frontierObserved(map, {boundary}));
  map.data.clear();
  EXPECT_FALSE(explore::frontierObserved(map, {boundary}));
  EXPECT_FALSE(explore::frontierObserved(map, {}));
}

TEST(FrontierObservation, ObservationDoesNotExhaustRetryBudget) {
  explore::FrontierRetry retry;
  geometry_msgs::msg::Point point;
  for (int i = 0; i < 10; ++i) retry.observed(point, i * 10.);
  EXPECT_TRUE(retry.blocked(point, 90.));
  EXPECT_FALSE(retry.blocked(point, 97.));
}

TEST(FrontierObservation, HandlesRotatedMapOrigin) {
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 2;
  map.info.height = 1;
  map.info.resolution = 1.;
  map.info.origin.orientation.z = std::sqrt(.5);
  map.info.origin.orientation.w = std::sqrt(.5);
  map.data = {0, -1};
  geometry_msgs::msg::Point point;
  point.x = -.5;
  point.y = 1.5;
  EXPECT_FALSE(explore::frontierObserved(map, {point}));
  map.data[1] = 0;
  EXPECT_TRUE(explore::frontierObserved(map, {point}));
}
