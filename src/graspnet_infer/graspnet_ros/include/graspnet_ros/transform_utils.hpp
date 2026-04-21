#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "graspnet_ros/graspnet_cpp.hpp"

namespace graspnet_ros {

/**
 * @brief Convert ROS TransformStamped to Eigen transformation matrix
 * @param transform ROS transform message
 * @return 4x4 transformation matrix
 */
Eigen::Matrix4f tfToEigenMatrix(const geometry_msgs::msg::TransformStamped& transform);

/**
 * @brief Convert Eigen rotation matrix to quaternion
 * @param rotation 3x3 rotation matrix
 * @return Quaternion (w, x, y, z)
 */
Eigen::Quaternionf rotationToQuaternion(const Eigen::Matrix3f& rotation);

/**
 * @brief Apply transformation to a single grasp
 * @param grasp Input grasp
 * @param transform_matrix 4x4 transformation matrix
 * @return Transformed grasp
 */
Grasp transformGrasp(const Grasp& grasp, const Eigen::Matrix4f& transform_matrix);

/**
 * @brief Apply transformation to multiple grasps
 * @param grasps Input grasps
 * @param transform_matrix 4x4 transformation matrix
 * @return Vector of transformed grasps
 */
std::vector<Grasp> transformGrasps(const std::vector<Grasp>& grasps, 
                                    const Eigen::Matrix4f& transform_matrix);

/**
 * @brief Transform vector of points using transformation matrix
 * @param points Input points
 * @param transform_matrix 4x4 transformation matrix
 */
void transformPoints(std::vector<Eigen::Vector3f>& points, 
                     const Eigen::Matrix4f& transform_matrix);

} // namespace graspnet_ros
