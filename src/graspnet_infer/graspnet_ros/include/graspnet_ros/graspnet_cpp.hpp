#pragma once

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <map>

// Constants
const int NUM_VIEWS = 300;
const int NUM_ANGLES = 12;
const int NUM_DEPTHS = 4;
const float GRASP_MAX_WIDTH = 0.1f;
const float GRASP_MAX_TOLERANCE = 0.05f;

struct Grasp {
    float score;
    float width;
    float height;
    float depth;
    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;
    int object_id;
};

// Function declarations
void load_data(const std::string& data_dir, 
               cv::Mat& color, cv::Mat& depth, cv::Mat& workspace_mask, 
               Eigen::Matrix3f& intrinsic, float& factor_depth);

struct PointCloud {
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> colors;
};

PointCloud create_point_cloud_from_depth(const cv::Mat& depth, const cv::Mat& color, 
                                         const cv::Mat& mask, const Eigen::Matrix3f& intrinsic, 
                                         float factor_depth);

void sample_points(PointCloud& cloud, int num_point, 
                   std::vector<Eigen::Vector3f>& sampled_points, 
                   std::vector<Eigen::Vector3f>& sampled_colors);

std::vector<Grasp> pred_decode(const std::map<std::string, std::vector<float>>& endpoints, int batch_size, int num_samples);

void nms_grasps(std::vector<Grasp>& grasps, float thres_dist, float thres_angle);

void save_scene_ply(const std::string& filename, const PointCloud& cloud, const std::vector<Grasp>& grasps);
