#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <string>
#include <yaml-cpp/yaml.h>

namespace seek_disparity {

// Configuration structures
struct TensorRTConfig {
    std::string engine_path;
    int batch_size;
    int camera_img_width;
    int camera_img_height;
    int cuda_device;
};

struct DepthProcessingConfig {
    bool use_confidence_filter;
    float filter_percentile;
    bool use_gradient_filter;
    float gradient_threshold;
    int gradient_dilation_size;
};

struct TopicsConfig {
    std::string left_image;
    std::string right_image;
    std::string disparity;
    std::string depth;
    std::string pointcloud;
    std::string confidence;
};

struct CameraIntrinsics {
    float fx;
    float fy;
    float cx;
    float cy;
    float baseline;
};

struct PointCloudConfig {
    bool enable;  // 是否生成点云
};

struct AlgorithmConfig {
    float min_depth;
    float max_depth;
};

struct DisparityConfig {
    TensorRTConfig tensorrt;
    DepthProcessingConfig depth_processing;
    TopicsConfig topics;
    CameraIntrinsics camera_intrinsics;
    PointCloudConfig pointcloud;
    AlgorithmConfig algorithm;
};

class ConfigManager {
public:
    ConfigManager();
    ~ConfigManager() = default;

    // Load configuration from YAML file
    bool loadConfig(const std::string& config_file_path);
    
    // Get configuration sections
    const TensorRTConfig& getTensorRTConfig() const { return config_.tensorrt; }
    const DepthProcessingConfig& getDepthProcessingConfig() const { return config_.depth_processing; }
    const TopicsConfig& getTopicsConfig() const { return config_.topics; }
    const PointCloudConfig& getPointCloudConfig() const { return config_.pointcloud; }
    const AlgorithmConfig& getAlgorithmConfig() const { return config_.algorithm; }
    const CameraIntrinsics& getCameraIntrinsics() const { return config_.camera_intrinsics; }

private:
    DisparityConfig config_;
    
    // Helper methods for parsing YAML
    bool parseTensorRTConfig(const YAML::Node& node);
    bool parseDepthProcessingConfig(const YAML::Node& node);
    bool parseTopicsConfig(const YAML::Node& node);
    bool parseCameraIntrinsics(const YAML::Node& node);
    bool parsePointCloudConfig(const YAML::Node& node);
    bool parseAlgorithmConfig(const YAML::Node& node);
};

} // namespace seek_disparity

#endif // CONFIG_MANAGER_H
