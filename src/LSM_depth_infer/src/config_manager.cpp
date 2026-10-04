#include "config_manager.h"
#include <iostream>
#include <fstream>

namespace seek_disparity {

ConfigManager::ConfigManager() : config_{} {
    // No default values - all parameters must be loaded from config file
}

bool ConfigManager::loadConfig(const std::string& config_file_path) {
    std::cout << "Loading configuration from: " << config_file_path << std::endl;
    
    try {
        YAML::Node config = YAML::LoadFile(config_file_path);
        
        // Parse TensorRT config
        if (config["tensorrt"]) {
            if (!parseTensorRTConfig(config["tensorrt"])) {
                std::cerr << "Failed to parse TensorRT config" << std::endl;
                return false;
            }
        }
        
        // Parse depth processing config
        if (config["depth_processing"]) {
            if (!parseDepthProcessingConfig(config["depth_processing"])) {
                std::cerr << "Failed to parse depth processing config" << std::endl;
                return false;
            }
        }
        
        // Parse topics config
        if (config["topics"]) {
            if (!parseTopicsConfig(config["topics"])) {
                std::cerr << "Failed to parse topics config" << std::endl;
                return false;
            }
        }
        
        // Parse camera intrinsics
        if (config["camera"] && config["camera"]["intrinsics"]) {
            if (!parseCameraIntrinsics(config["camera"]["intrinsics"])) {
                std::cerr << "Failed to parse camera intrinsics" << std::endl;
                return false;
            }
        }
        
        // Parse pointcloud config
        if (config["pointcloud"]) {
            if (!parsePointCloudConfig(config["pointcloud"])) {
                std::cerr << "Failed to parse pointcloud config" << std::endl;
                return false;
            }
        }
        
        // Parse algorithm config
        if (config["algorithm"]) {
            if (!parseAlgorithmConfig(config["algorithm"])) {
                std::cerr << "Failed to parse algorithm config" << std::endl;
                return false;
            }
        }
        
        std::cout << "Configuration loaded successfully" << std::endl;
        return true;
        
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return false;
    }
}

bool ConfigManager::parseTensorRTConfig(const YAML::Node& node) {
    if (node["engine_path"]) {
        config_.tensorrt.engine_path = node["engine_path"].as<std::string>();
    }
    if (node["batch_size"]) {
        config_.tensorrt.batch_size = node["batch_size"].as<int>();
    }
    if (node["camera_img_width"]) {
        config_.tensorrt.camera_img_width = node["camera_img_width"].as<int>();
    }
    if (node["camera_img_height"]) {
        config_.tensorrt.camera_img_height = node["camera_img_height"].as<int>();
    }
    if (node["cuda_device"]) {
        config_.tensorrt.cuda_device = node["cuda_device"].as<int>();
    }
    
    std::cout << "TensorRT config: engine=" << config_.tensorrt.engine_path 
              << ", batch=" << config_.tensorrt.batch_size
              << ", size=" << config_.tensorrt.camera_img_width << "x" << config_.tensorrt.camera_img_height
              << std::endl;
    return true;
}

bool ConfigManager::parseDepthProcessingConfig(const YAML::Node& node) {
    if (node["use_confidence_filter"]) {
        config_.depth_processing.use_confidence_filter = node["use_confidence_filter"].as<bool>();
    }
    if (node["filter_percentile"]) {
        config_.depth_processing.filter_percentile = node["filter_percentile"].as<float>();
    }
    if (node["use_gradient_filter"]) {
        config_.depth_processing.use_gradient_filter = node["use_gradient_filter"].as<bool>();
    }
    if (node["gradient_threshold"]) {
        config_.depth_processing.gradient_threshold = node["gradient_threshold"].as<float>();
    }
    if (node["gradient_dilation_size"]) {
        config_.depth_processing.gradient_dilation_size = node["gradient_dilation_size"].as<int>();
    }
    return true;
}

bool ConfigManager::parseTopicsConfig(const YAML::Node& node) {
    if (node["left_image"]) {
        config_.topics.left_image = node["left_image"].as<std::string>();
    }
    if (node["right_image"]) {
        config_.topics.right_image = node["right_image"].as<std::string>();
    }
    if (node["disparity"]) {
        config_.topics.disparity = node["disparity"].as<std::string>();
    }
    if (node["depth"]) {
        config_.topics.depth = node["depth"].as<std::string>();
    }
    if (node["pointcloud"]) {
        config_.topics.pointcloud = node["pointcloud"].as<std::string>();
    }
    if (node["confidence"]) {
        config_.topics.confidence = node["confidence"].as<std::string>();
    }
    return true;
}

bool ConfigManager::parseCameraIntrinsics(const YAML::Node& node) {
    if (node["fx"]) {
        config_.camera_intrinsics.fx = node["fx"].as<float>();
    }
    if (node["fy"]) {
        config_.camera_intrinsics.fy = node["fy"].as<float>();
    }
    if (node["cx"]) {
        config_.camera_intrinsics.cx = node["cx"].as<float>();
    }
    if (node["cy"]) {
        config_.camera_intrinsics.cy = node["cy"].as<float>();
    }
    if (node["baseline"]) {
        config_.camera_intrinsics.baseline = node["baseline"].as<float>();
    }
    
    std::cout << "Camera intrinsics: fx=" << config_.camera_intrinsics.fx 
              << ", fy=" << config_.camera_intrinsics.fy
              << ", cx=" << config_.camera_intrinsics.cx
              << ", cy=" << config_.camera_intrinsics.cy
              << ", baseline=" << config_.camera_intrinsics.baseline
              << std::endl;
    return true;
}

bool ConfigManager::parsePointCloudConfig(const YAML::Node& node) {
    if (node["enable"]) {
        config_.pointcloud.enable = node["enable"].as<bool>();
    }
    std::cout << "Pointcloud config: enable=" << (config_.pointcloud.enable ? "true" : "false") << std::endl;
    return true;
}

bool ConfigManager::parseAlgorithmConfig(const YAML::Node& node) {
    if (node["min_depth"]) {
        config_.algorithm.min_depth = node["min_depth"].as<float>();
    }
    if (node["max_depth"]) {
        config_.algorithm.max_depth = node["max_depth"].as<float>();
    }
    std::cout << "Algorithm config: min_depth=" << config_.algorithm.min_depth 
              << ", max_depth=" << config_.algorithm.max_depth << std::endl;
    return true;
}

} // namespace seek_disparity
