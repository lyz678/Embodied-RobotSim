#pragma once

#include <vector>
#include <Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/header.hpp>
#include "graspnet_ros/graspnet_cpp.hpp"

namespace graspnet_ros {

/**
 * @brief Convert sampled points to ROS PointCloud2 message
 * @param points 3D points
 * @param colors RGB colors (0-1 range)
 * @param header ROS message header
 * @param target_frame Target frame ID for the point cloud
 * @return PointCloud2 message
 */
sensor_msgs::msg::PointCloud2 createPointCloudMsg(
    const std::vector<Eigen::Vector3f>& points,
    const std::vector<Eigen::Vector3f>& colors,
    const std_msgs::msg::Header& header,
    const std::string& target_frame
);

/**
 * @brief Create gripper visualization markers for grasps
 * @param grasps Vector of grasps
 * @param header ROS message header
 * @param namespace_prefix Marker namespace prefix
 * @param is_matched Whether these are matched grasps (affects color)
 * @param max_count Maximum number of grasps to visualize
 * @return MarkerArray message
 */
visualization_msgs::msg::MarkerArray createGraspMarkers(
    const std::vector<Grasp>& grasps,
    const std_msgs::msg::Header& header,
    const std::string& namespace_prefix = "grasps",
    bool is_matched = false,
    int max_count = 10
);

/**
 * @brief Create coordinate frame axes markers for grasps
 * @param grasps Vector of grasps
 * @param header ROS message header
 * @param axis_length Length of each axis in meters
 * @param line_width Width of axis lines
 * @return MarkerArray message
 */
visualization_msgs::msg::MarkerArray createGraspAxesMarkers(
    const std::vector<Grasp>& grasps,
    const std_msgs::msg::Header& header,
    float axis_length = 0.1f,
    float line_width = 0.005f
);

/**
 * @brief Convert grasps to PoseArray message
 * @param grasps Vector of grasps
 * @param header ROS message header
 * @return PoseArray message
 */
geometry_msgs::msg::PoseArray createPoseArray(
    const std::vector<Grasp>& grasps,
    const std_msgs::msg::Header& header
);

} // namespace graspnet_ros
