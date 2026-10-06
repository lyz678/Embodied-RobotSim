#pragma once

#include <vector>
#include <Eigen/Dense>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include "graspnet_ros/graspnet_cpp.hpp"

namespace graspnet_ros {

/**
 * @brief Correct grasp orientation to ensure approach vector points downward
 *        and Z-axis points upward in global frame
 * @param grasp Input grasp to correct
 * @return Corrected grasp
 */
Grasp correctGraspOrientation(const Grasp& grasp);

/**
 * @brief Apply orientation correction to multiple grasps
 * @param grasps Input grasps
 */
void correctGraspOrientations(std::vector<Grasp>& grasps);

/**
 * @brief Filter grasps by minimum score threshold
 * @param grasps Input grasps
 * @param min_score Minimum score threshold
 * @return Filtered grasps
 */
std::vector<Grasp> filterGraspsByScore(const std::vector<Grasp>& grasps, float min_score);

/**
 * @brief Match grasps to detections based on spatial proximity
 * @param grasps Available grasps
 * @param detections Object detections
 * @param max_distance Maximum distance for matching (meters)
 * @param min_score Minimum grasp score to consider
 * @return Vector of matched grasps (same order as detections, empty Grasp if no match)
 */
std::vector<Grasp> matchGraspsToDetections(
    const std::vector<Grasp>& grasps,
    const vision_msgs::msg::Detection3DArray& detections,
    float max_distance = 0.1f,
    float min_score = 0.5f
);

/**
 * @brief Update Detection3D poses with matched grasp poses
 * @param detections Input detections (will be modified)
 * @param matched_grasps Matched grasps for each detection
 */
void updateDetectionPoses(
    vision_msgs::msg::Detection3DArray& detections,
    const std::vector<Grasp>& matched_grasps
);

} // namespace graspnet_ros
