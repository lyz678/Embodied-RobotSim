#include "graspnet_ros/graspnet_cpp.hpp"
#include <fstream>
#include <iostream>
#include <random>
#include <algorithm>
#include <cmath>

void sample_points(PointCloud& cloud, int num_point, 
                   std::vector<Eigen::Vector3f>& sampled_points, 
                   std::vector<Eigen::Vector3f>& sampled_colors) {
    int N = cloud.points.size();
    if (N == 0) return;

    std::vector<int> indices(N);
    std::iota(indices.begin(), indices.end(), 0);

    // Use fixed seed for reproducible results comparison with Python
    // Use random_device for non-deterministic seed matching Python's default behavior
    std::random_device rd;
    std::mt19937 g(rd()); 
    
    sampled_points.resize(num_point);
    sampled_colors.resize(num_point);

    if (N >= num_point) {
        std::shuffle(indices.begin(), indices.end(), g);
        for (int i = 0; i < num_point; ++i) {
            sampled_points[i] = cloud.points[indices[i]];
            sampled_colors[i] = cloud.colors[indices[i]];
        }
    } else {
        // With replacement fill
        for (int i = 0; i < N; ++i) {
            sampled_points[i] = cloud.points[indices[i]];
            sampled_colors[i] = cloud.colors[indices[i]];
        }
        std::uniform_int_distribution<> dis(0, N - 1);
        for (int i = N; i < num_point; ++i) {
            int idx = dis(g);
            sampled_points[i] = cloud.points[indices[idx]];
            sampled_colors[i] = cloud.colors[indices[idx]];
        }
    }
}

// --- Post-processing ---

// Helper: Batch Viewpoint to Matrix
// approaches: Nx3, angles: N
Eigen::Matrix3f viewpoint_to_matrix(const Eigen::Vector3f& towards, float angle) {
    Eigen::Vector3f axis_x = towards.normalized();
    Eigen::Vector3f axis_y;
    
    if (std::abs(axis_x[0]) < std::abs(axis_x[1])) {
        axis_y = Eigen::Vector3f(-axis_x[1], axis_x[0], 0);
    } else {
        // Only if x and y are both 0 (impossible for normalized vector) or generic case
        // The python code: [-axis_x[:,1], axis_x[:,0], zeros]
        // If axis_x[0] and axis_x[1] are 0, this is 0 vector.
        // Python handles this: mask_y = (norm(axis_y)==0) -> axis_y[mask, 1] = 1.
        axis_y = Eigen::Vector3f(-axis_x[1], axis_x[0], 0);
    }
    
    if (axis_y.norm() < 1e-6) {
        axis_y = Eigen::Vector3f(0, 1, 0);
    }
    axis_y.normalize();
    Eigen::Vector3f axis_z = axis_x.cross(axis_y);
    
    float s = std::sin(angle);
    float c = std::cos(angle);
    
    Eigen::Matrix3f R1;
    R1 << 1, 0, 0,
          0, c, -s,
          0, s, c;
          
    Eigen::Matrix3f R2;
    R2.col(0) = axis_x;
    R2.col(1) = axis_y;
    R2.col(2) = axis_z;
    
    return R2 * R1;
}

std::vector<Grasp> pred_decode(const std::map<std::string, std::vector<float>>& endpoints, int batch_size, int num_samples) {
    // Assumptions on tensor layout based on Python analysis:
    // objectness_score: [2, N, 1] (or similar) - Actually [2, N] likely?
    // grasp_score_pred: [12, N, 4] (A, N, D)
    // grasp_width_pred: [12, N, 4]
    // ...
    // Note: Vectors are flat. We need to index them correctly.
    // Let's assume N = num_samples (20000). A=12, D=4.
    
    // Check sizes
    // int N = num_samples;
    // int A = 12;
    // int D = 4;
    
    // float* p_score = endpoints.at("grasp_score_pred").data();
    // ...
    
    // To handle this generically without a tensor library is tedious.
    // Implementation Strategy:
    // Iterate over N points. For each point:
    // 1. Get objectness score. If Object, proceed.
    // 2. Find best Angle (argmax over 12 angles) for each Depth? 
    //    Python: argmax over Angle first. 
    //    For a point i: score[a, i, d]. Maximize over 'a' -> best_a[i, d].
    //    Then maximize over 'd' -> best_d[i].
    //    So we find best (a, d) for point i.
    // 3. Extract properties.
    // 4. Compute Rotation.
    
    std::vector<Grasp> grasps;
    
    const auto& score_vec = endpoints.at("grasp_score_pred");
    const auto& width_vec = endpoints.at("grasp_width_pred");
    const auto& angle_cls_vec = endpoints.at("grasp_angle_cls_pred");
    const auto& tolerance_vec = endpoints.at("grasp_tolerance_pred");
    const auto& obj_score_vec = endpoints.at("objectness_score");
    const auto& center_vec = endpoints.at("grasp_center");
    const auto& approach_vec = endpoints.at("approaching");
    
    // Dimensions
    int N = num_samples; // Use dynamic output size
    if (N == 0) N = 1024; // Fallback if 0 passed
    int A = 12;
    int D = 4;
    
    for (int i = 0; i < N; ++i) {
        // Objectness: Standard argmax over [2, N]
        float s0 = obj_score_vec[i];
        float s1 = obj_score_vec[N + i];
        if (s1 <= s0) continue; // Not object
        
        // 1. For each depth, find the best angle according to angle_cls_pred
        std::vector<int> best_a_for_d(D);
        for (int d = 0; d < D; ++d) {
            int best_a = -1;
            float max_a_score = -1e10;
            for (int a = 0; a < A; ++a) {
                float val = angle_cls_vec[a * (N * D) + i * D + d];
                if (val > max_a_score) {
                    max_a_score = val;
                    best_a = a;
                }
            }
            best_a_for_d[d] = best_a;
        }
        
        // 2. Find the best depth among the selected angles using grasp_score_pred
        int best_d = -1;
        float max_score = -1e10;
        for (int d = 0; d < D; ++d) {
            int a = best_a_for_d[d];
            float val = score_vec[a * (N * D) + i * D + d];
            if (val > max_score) {
                max_score = val;
                best_d = d;
            }
        }
        
        int best_a = best_a_for_d[best_d];
        int idx = best_a * (N * D) + i * D + best_d;
        
        // Extract features
        float raw_score = score_vec[idx];
        float width = 1.2f * width_vec[idx];
        width = std::min(std::max(width, 0.0f), GRASP_MAX_WIDTH);
        float tolerance = tolerance_vec[idx];
        
        // Score Scaling: score * tolerance / max_tolerance (0.05)
        float scaled_score = raw_score * tolerance / GRASP_MAX_TOLERANCE;
        
        // Geometry
        float cx = center_vec[i * 3 + 0];
        float cy = center_vec[i * 3 + 1];
        float cz = center_vec[i * 3 + 2];
        
        float ax = -approach_vec[i * 3 + 0];
        float ay = -approach_vec[i * 3 + 1];
        float az = -approach_vec[i * 3 + 2];
        
        float angle = (float)best_a / 12.0f * M_PI;
        Eigen::Vector3f approach(ax, ay, az);
        Eigen::Matrix3f rot = viewpoint_to_matrix(approach, angle);
        
        Grasp g;
        g.score = scaled_score;
        g.width = width;
        g.height = 0.02f;
        g.depth = (best_d + 1) * 0.01f;
        g.rotation = rot;
        g.translation = Eigen::Vector3f(cx, cy, cz);
        g.object_id = -1;
        
        grasps.push_back(g);
    }
    
    return grasps;
}

void nms_grasps(std::vector<Grasp>& grasps, float thres_dist, float thres_angle) {
    // Basic NMS: Sort by score first
    std::sort(grasps.begin(), grasps.end(), [](const Grasp& a, const Grasp& b) {
        return a.score > b.score;
    });
    
    std::vector<bool> suppressed(grasps.size(), false);
    std::vector<Grasp> result;
    
    for (size_t i = 0; i < grasps.size(); ++i) {
        if (suppressed[i]) continue;
        result.push_back(grasps[i]);
        
        for (size_t j = i + 1; j < grasps.size(); ++j) {
            if (suppressed[j]) continue;
            
            // Distance check
            float dist = (grasps[i].translation - grasps[j].translation).norm();
            if (dist < thres_dist) {
                // Rotation check
                // Trace(R1 * R2^T)
                // angle = arccos((trace - 1) / 2)
                float trace = (grasps[i].rotation * grasps[j].rotation.transpose()).trace();
                float val = (trace - 1.0f) / 2.0f;
                val = std::min(std::max(val, -1.0f), 1.0f); // Clamp for numerical stability
                float angle_diff = std::acos(val);
                
                if (angle_diff < thres_angle) {
                    suppressed[j] = true; 
                }
            }
        }
    }
    grasps = result;
}
