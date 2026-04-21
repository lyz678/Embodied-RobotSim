#include "graspnet_ros/grasp_processing.hpp"
#include <cmath>
#include <limits>

namespace graspnet_ros {

Grasp correctGraspOrientation(const Grasp& grasp) {
    Grasp corrected = grasp;
    
    // Flip upward facing grasps to downward (Approach Vector Z > 0)
    // Approach vector is Column 0 of rotation matrix
    if (corrected.rotation(2, 0) > 0.05) { // Threshold 0.05 to avoid jitter near horizontal
        Eigen::Matrix3f M = Eigen::Vector3f(1, 1, -1).asDiagonal();
        Eigen::Matrix3f F = Eigen::Vector3f(1, -1, 1).asDiagonal();
        corrected.rotation = M * corrected.rotation * F;
    }
    
    // Force Z-axis (Blue) to point Upward (Global Z > 0)
    // Useful if Z-axis represents Top/Normal orientation
    if (corrected.rotation(2, 2) < 0) { // If Z-axis points down
        // Rotate 180 deg around Local X (Approach)
        // This flips Z and Y, keeping X (Approach) constant
        Eigen::Matrix3f RotX180 = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix();
        corrected.rotation = corrected.rotation * RotX180;
    }
    
    return corrected;
}

void correctGraspOrientations(std::vector<Grasp>& grasps) {
    for (auto& grasp : grasps) {
        grasp = correctGraspOrientation(grasp);
    }
}

std::vector<Grasp> filterGraspsByScore(const std::vector<Grasp>& grasps, float min_score) {
    std::vector<Grasp> filtered;
    filtered.reserve(grasps.size());
    
    for (const auto& grasp : grasps) {
        if (grasp.score >= min_score) {
            filtered.push_back(grasp);
        }
    }
    
    return filtered;
}

std::vector<Grasp> matchGraspsToDetections(
    const std::vector<Grasp>& grasps,
    const vision_msgs::msg::Detection3DArray& detections,
    float max_distance,
    float min_score
) {
    std::vector<Grasp> matched_grasps;
    matched_grasps.reserve(detections.detections.size());
    
    // Iterate through detections
    for (const auto& det : detections.detections) {
        // Center of detection
        Eigen::Vector3f center(
            det.bbox.center.position.x,
            det.bbox.center.position.y,
            det.bbox.center.position.z
        );
        
        // Find nearest grasp
        float min_dist = std::numeric_limits<float>::max();
        int best_idx = -1;
        
        for (size_t i = 0; i < grasps.size(); ++i) {
            // Filter by score first to avoid bad grasps
            if (grasps[i].score < min_score) continue;
            
            float dist = (grasps[i].translation - center).norm();
            
            // Check distance threshold
            if (dist < max_distance && dist < min_dist) {
                min_dist = dist;
                best_idx = i;
            }
        }
        
        // Add matched grasp or empty grasp
        if (best_idx != -1) {
            matched_grasps.push_back(grasps[best_idx]);
        } else {
            // Create empty grasp as placeholder
            Grasp empty;
            empty.score = -1.0f;  // Use negative score to indicate no match
            matched_grasps.push_back(empty);
        }
    }
    
    return matched_grasps;
}

void updateDetectionPoses(
    vision_msgs::msg::Detection3DArray& detections,
    const std::vector<Grasp>& matched_grasps
) {
    for (size_t i = 0; i < detections.detections.size() && i < matched_grasps.size(); ++i) {
        const auto& grasp = matched_grasps[i];
        
        // Skip if no match (negative score)
        if (grasp.score < 0) continue;
        
        auto& det = detections.detections[i];
        
        // Update detection pose with grasp pose
        if (!det.results.empty()) {
            Eigen::Quaternionf q(grasp.rotation);
            det.results[0].pose.pose.position.x = grasp.translation.x();
            det.results[0].pose.pose.position.y = grasp.translation.y();
            det.results[0].pose.pose.position.z = grasp.translation.z();
            det.results[0].pose.pose.orientation.w = q.w();
            det.results[0].pose.pose.orientation.x = q.x();
            det.results[0].pose.pose.orientation.y = q.y();
            det.results[0].pose.pose.orientation.z = q.z();
            
            // Also update score
            det.results[0].hypothesis.score = grasp.score;
        }
    }
}

} // namespace graspnet_ros
