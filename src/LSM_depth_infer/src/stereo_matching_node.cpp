/**
 * @file stereo_matching_node.cpp
 * @brief ROS2 node for stereo depth inference using TensorRT
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_srvs/srv/set_bool.hpp>
#include <future>
#include <atomic>

#include "inference.h"
#include "config_manager.h"
#include "pointcloud_generator.cuh"

using namespace std::chrono_literals;

// 优化的置信度过滤函数 - 单次循环完成阈值计算和过滤
void applyConfidenceFilterOptimized(cv::Mat& disparity, const cv::Mat& confidence, float percentile) {
    CV_Assert(disparity.size() == confidence.size());
    CV_Assert(disparity.type() == CV_32F && confidence.type() == CV_32F);
    
    // 第一步：收集有效像素的置信度
    std::vector<float> valid_confidences;
    valid_confidences.reserve(disparity.rows * disparity.cols / 2); // 预分配内存
    
    const int rows = disparity.rows;
    const int cols = disparity.cols;
    
    for (int v = 0; v < rows; ++v) {
        const float* disp_ptr = disparity.ptr<float>(v);
        const float* conf_ptr = confidence.ptr<float>(v);
        
        for (int u = 0; u < cols; ++u) {
            if (disp_ptr[u] > 0.001f) {
                valid_confidences.push_back(conf_ptr[u]);
            }
        }
    }
    
    if (valid_confidences.empty()) {
        return;
    }
    
    // 第二步：计算阈值
    size_t n = valid_confidences.size();
    size_t k = static_cast<size_t>(percentile * n / 100.0);
    k = std::min(k, n - 1);
    
    std::nth_element(valid_confidences.begin(), 
                    valid_confidences.begin() + k, 
                    valid_confidences.end());
    
    float threshold = valid_confidences[k];
    
    // 第三步：在同一次循环中应用过滤
    for (int v = 0; v < rows; ++v) {
        float* disp_ptr = disparity.ptr<float>(v);
        const float* conf_ptr = confidence.ptr<float>(v);
        
        for (int u = 0; u < cols; ++u) {
            if (conf_ptr[u] < threshold || disp_ptr[u] <= 0.001f) {
                disp_ptr[u] = 0.0f;
            }
        }
    }
}

// 合并的置信度+梯度滤波函数 - 优化为减少遍历次数
void applyConfidenceAndGradientFilter(cv::Mat& disparity, const cv::Mat& confidence, 
                                      float conf_percentile, float grad_threshold, 
                                      int dilation_size) {
    CV_Assert(disparity.size() == confidence.size());
    CV_Assert(disparity.type() == CV_32F && confidence.type() == CV_32F);
    
    const int rows = disparity.rows;
    const int cols = disparity.cols;
    
    // 第一步：收集有效像素的置信度 + 计算梯度
    std::vector<float> valid_confidences;
    valid_confidences.reserve(rows * cols / 2);
    
    // 计算深度图的梯度
    cv::Mat grad_x, grad_y, grad_magnitude;
    cv::Sobel(disparity, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(disparity, grad_y, CV_32F, 0, 1, 3);
    cv::magnitude(grad_x, grad_y, grad_magnitude);
    
    // 同时收集置信度和标记高梯度区域
    cv::Mat high_gradient_mask = cv::Mat::zeros(disparity.size(), CV_8U);
    
    for (int v = 0; v < rows; ++v) {
        const float* disp_ptr = disparity.ptr<float>(v);
        const float* conf_ptr = confidence.ptr<float>(v);
        const float* grad_ptr = grad_magnitude.ptr<float>(v);
        uchar* mask_ptr = high_gradient_mask.ptr<uchar>(v);
        
        for (int u = 0; u < cols; ++u) {
            float d = disp_ptr[u];
            if (d > 0.001f) {
                // 收集置信度
                valid_confidences.push_back(conf_ptr[u]);
                
                // 检查梯度
                if (std::isfinite(d) && grad_ptr[u] > grad_threshold) {
                    mask_ptr[u] = 255;
                }
            }
        }
    }
    
    // 第二步：计算置信度阈值
    float conf_thresh = 0.0f;
    if (!valid_confidences.empty()) {
        size_t n = valid_confidences.size();
        size_t k = static_cast<size_t>(conf_percentile * n / 100.0);
        k = std::min(k, n - 1);
        
        std::nth_element(valid_confidences.begin(), 
                        valid_confidences.begin() + k, 
                        valid_confidences.end());
        conf_thresh = valid_confidences[k];
    }
    
    // 第三步：膨胀高梯度区域
    if (dilation_size > 0) {
        int kernel_size = 2 * dilation_size + 1;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                                   cv::Size(kernel_size, kernel_size));
        cv::dilate(high_gradient_mask, high_gradient_mask, kernel);
    }
    
    // 第四步：合并应用置信度和梯度滤波
    for (int v = 0; v < rows; ++v) {
        float* disp_ptr = disparity.ptr<float>(v);
        const float* conf_ptr = confidence.ptr<float>(v);
        const uchar* mask_ptr = high_gradient_mask.ptr<uchar>(v);
        
        for (int u = 0; u < cols; ++u) {
            // 置信度过滤或梯度过滤
            if (conf_ptr[u] < conf_thresh || disp_ptr[u] <= 0.001f || mask_ptr[u] > 0) {
                disp_ptr[u] = 0.0f;
            }
        }
    }
}

class StereoMatchingNode : public rclcpp::Node
{
public:
    StereoMatchingNode() : Node("stereo_matching_node")
    {
        // These options configure buffers/topics at startup; expose them as
        // read-only ROS parameters rather than accepting ineffective live sets.
        auto option = [this](const std::string& name, auto value) {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.read_only = true;
            return this->declare_parameter(name, value, descriptor);
        };
        option("config_file", std::string(""));
        
        // Get config file path
        std::string config_file = this->get_parameter("config_file").as_string();
        
        // If not specified, use default
        if (config_file.empty()) {
            std::string pkg_dir = ament_index_cpp::get_package_share_directory("stereo_matching");
            config_file = pkg_dir + "/config/config.yaml";
        }
        
        RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());
        
        // Load configuration
        config_manager_ = std::make_unique<seek_disparity::ConfigManager>();
        if (!config_manager_->loadConfig(config_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load configuration file");
            throw std::runtime_error("Failed to load configuration");
        }
        
        // Get config values
        const auto& tensorrt_config = config_manager_->getTensorRTConfig();
        auto topics_config = config_manager_->getTopicsConfig();
        const auto& algorithm_config = config_manager_->getAlgorithmConfig();
        const auto& camera_config = config_manager_->getCameraIntrinsics();
        const auto& depth_processing_config = config_manager_->getDepthProcessingConfig();
        
        // ROS overrides take precedence over values in config_file.
        fx_ = option("camera.fx", double(camera_config.fx));
        fy_ = option("camera.fy", double(camera_config.fy));
        cx_ = option("camera.cx", double(camera_config.cx));
        cy_ = option("camera.cy", double(camera_config.cy));
        baseline_ = option("camera.baseline", double(camera_config.baseline));
        
        // Store algorithm parameters
        min_depth_ = option("min_depth", double(algorithm_config.min_depth));
        max_depth_ = option("max_depth", double(algorithm_config.max_depth));
        
        // Store depth processing parameters
        use_confidence_filter_ = option("use_confidence_filter", depth_processing_config.use_confidence_filter);
        filter_percentile_ = option("filter_percentile", double(depth_processing_config.filter_percentile));
        use_gradient_filter_ = option("use_gradient_filter", depth_processing_config.use_gradient_filter);
        gradient_threshold_ = option("gradient_threshold", double(depth_processing_config.gradient_threshold));
        gradient_dilation_size_ = option("gradient_dilation_size", depth_processing_config.gradient_dilation_size);
        
        // Store pointcloud config
        const auto& pointcloud_config = config_manager_->getPointCloudConfig();
        enable_pointcloud_ = option("enable_pointcloud", pointcloud_config.enable);
        
        // Store inference size from config
        inference_width_ = tensorrt_config.camera_img_width;
        inference_height_ = tensorrt_config.camera_img_height;
        
        // Initialize TensorRT inference engine
        for (auto entry : {std::pair<const char*, std::string*>{"left_image", &topics_config.left_image},
                {"right_image", &topics_config.right_image}, {"depth", &topics_config.depth},
                {"pointcloud", &topics_config.pointcloud}, {"disparity", &topics_config.disparity},
                {"confidence", &topics_config.confidence}}) {
            *entry.second = option(std::string("topics.") + entry.first, *entry.second);
        }
        std::string engine_path = option("engine_path", tensorrt_config.engine_path);
        if (engine_path.empty() || !std::isfinite(fx_) || fx_ <= 0 || !std::isfinite(fy_) || fy_ <= 0 ||
            !std::isfinite(baseline_) || baseline_ <= 0 || !std::isfinite(min_depth_) || min_depth_ <= 0 ||
            !std::isfinite(max_depth_) || max_depth_ <= min_depth_ ||
            !std::isfinite(filter_percentile_) || filter_percentile_ < 0 || filter_percentile_ > 100 ||
            !std::isfinite(gradient_threshold_) || gradient_threshold_ < 0 || gradient_dilation_size_ < 0) {
            throw std::runtime_error("Invalid LSM calibration/depth/filter/engine configuration");
        }
        // If relative path, resolve from package share directory
        if (engine_path[0] != '/') {
            std::string pkg_dir = ament_index_cpp::get_package_share_directory("stereo_matching");
            engine_path = pkg_dir + "/" + engine_path;
        }
        
        RCLCPP_INFO(this->get_logger(), "Initializing TensorRT with engine: %s", engine_path.c_str());
        
        try {
            infer_engine_ = std::make_unique<TrtInferCore>(engine_path, config_manager_.get());
            if (!infer_engine_->initialize()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize TensorRT engine");
                throw std::runtime_error("Failed to initialize TensorRT");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "TensorRT initialization error: %s", e.what());
            throw;
        }
        
        RCLCPP_INFO(this->get_logger(), "TensorRT engine initialized successfully");
        
        // Warm-up inference to avoid first-frame latency
        RCLCPP_INFO(this->get_logger(), "Performing TensorRT warm-up...");
        infer_engine_->warmup(3);  // Run 3 warm-up iterations
        RCLCPP_INFO(this->get_logger(), "TensorRT warm-up completed");
        
        // Set up QoS for image topics
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        
        // Create synchronized subscribers for stereo images
        left_sub_.subscribe(this, topics_config.left_image, qos.get_rmw_qos_profile());
        right_sub_.subscribe(this, topics_config.right_image, qos.get_rmw_qos_profile());
        
        // Use approximate time synchronization
        sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), left_sub_, right_sub_);
        sync_->registerCallback(std::bind(&StereoMatchingNode::stereoCallback, this, 
                                          std::placeholders::_1, std::placeholders::_2));
        
        // Create publishers
        disparity_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topics_config.disparity, 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topics_config.depth, 10);
        
        // Create confidence visualization publisher
        confidence_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topics_config.confidence, 10);
        
        // Initialize point cloud generator only if enabled
        if (enable_pointcloud_) {
            pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                topics_config.pointcloud, 10);
            pointcloud_generator_ = std::make_unique<CUDAPointCloudGenerator>();
        }
        
        // Initialize inference control
        this->declare_parameter("enable_inference", true);
        enable_inference_ = this->get_parameter("enable_inference").as_bool();
        
        // Create service to control inference
        srv_enable_inference_ = this->create_service<std_srvs::srv::SetBool>(
            "~/enable_inference",
            std::bind(&StereoMatchingNode::enable_inference_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Inference control initialized (enabled=%s)", 
                    enable_inference_ ? "true" : "false");
        
        RCLCPP_INFO(this->get_logger(), "Stereo matching node initialized (pointcloud=%s)", 
                    enable_pointcloud_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s, %s", 
                    topics_config.left_image.c_str(), topics_config.right_image.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: disparity=%s, depth=%s%s",
                    topics_config.disparity.c_str(), topics_config.depth.c_str(),
                    enable_pointcloud_ ? (", pointcloud=" + topics_config.pointcloud).c_str() : "");
    }

private:
    
    void stereoCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
    {
        // Skip processing if inference is disabled
        if (!enable_inference_) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "Inference disabled, skipping stereo pair");
            return;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Received stereo pair");
        
        try {
            // Convert ROS images to OpenCV
            cv_bridge::CvImagePtr left_cv = cv_bridge::toCvCopy(left_msg, "bgr8");
            cv_bridge::CvImagePtr right_cv = cv_bridge::toCvCopy(right_msg, "bgr8");
            
            // Run inference (includes preprocess + TensorRT inference + postprocess)
            cv::Mat disparity, confidence, resized_rgb;
            float scale;
            auto infer_start = std::chrono::high_resolution_clock::now();
            if (!infer_engine_->infer(left_cv->image, right_cv->image, disparity, confidence, resized_rgb, scale)) {
                RCLCPP_ERROR(this->get_logger(), "Inference failed");
                return;
            }
            auto infer_end = std::chrono::high_resolution_clock::now();
            
            if (disparity.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Empty disparity output");
                return;
            }
            
            // Apply confidence and/or gradient filtering to disparity
            if (use_confidence_filter_ && use_gradient_filter_) {
                // 合并滤波：同时应用置信度和梯度过滤
                applyConfidenceAndGradientFilter(disparity, confidence, filter_percentile_, 
                                                 gradient_threshold_, gradient_dilation_size_);
            } else if (use_confidence_filter_) {
                // 仅置信度过滤
                applyConfidenceFilterOptimized(disparity, confidence, filter_percentile_);
            } else if (use_gradient_filter_) {
                // 仅梯度过滤（使用置信度百分位0表示不过滤置信度）
                applyConfidenceAndGradientFilter(disparity, confidence, 0.0f, 
                                                 gradient_threshold_, gradient_dilation_size_);
            }
            
            // Publish confidence visualization (grayscale)
            {
                cv::Mat conf_normalized;
                cv::normalize(confidence, conf_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
                
                cv_bridge::CvImage conf_msg;
                conf_msg.header = left_msg->header;
                conf_msg.encoding = sensor_msgs::image_encodings::MONO8;
                conf_msg.image = conf_normalized;
                confidence_pub_->publish(*conf_msg.toImageMsg());
            }
            
            // Publish filtered disparity
            cv_bridge::CvImage disparity_msg;
            disparity_msg.header = left_msg->header;
            disparity_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            disparity_msg.image = disparity;
            disparity_pub_->publish(*disparity_msg.toImageMsg());
            
            // Compute scaled intrinsics for depth calculation
            float scaled_fx = fx_ * scale;
            float scaled_fy = fy_ * scale;
            float scaled_cx = cx_ * scale;
            float scaled_cy = cy_ * scale;
            
            // Convert disparity to depth: depth = baseline * scaled_fx / disparity
            cv::Mat depth = cv::Mat::zeros(disparity.size(), CV_32F);
            const int rows = disparity.rows;
            const int cols = disparity.cols;
            for (int v = 0; v < rows; ++v) {
                const float* disp_ptr = disparity.ptr<float>(v);
                float* depth_ptr = depth.ptr<float>(v);
                for (int u = 0; u < cols; ++u) {
                    if (disp_ptr[u] > 0.001f) {
                        float d = baseline_ * scaled_fx / disp_ptr[u];
                        if (d >= min_depth_ && d <= max_depth_) {
                            depth_ptr[u] = d;
                        }
                    }
                }
            }
            
            // Publish depth image
            cv_bridge::CvImage depth_msg;
            depth_msg.header = left_msg->header;
            depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            depth_msg.image = depth;
            depth_pub_->publish(*depth_msg.toImageMsg());
            
            // Calculate inference time
            double infer_ms = std::chrono::duration_cast<std::chrono::microseconds>(infer_end - infer_start).count() / 1000.0;
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Infer=%.2fms", infer_ms);
            
            // Generate and publish point cloud asynchronously (if enabled)
            if (enable_pointcloud_) {
                // Check if previous async task is still running
                if (pointcloud_future_.valid() && 
                    pointcloud_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                         "Skipping point cloud generation - previous task still running");
                } else {
                    // Clone data for async processing
                    cv::Mat depth_clone = depth.clone();
                    cv::Mat rgb_clone = resized_rgb.clone();
                    std_msgs::msg::Header header_copy = left_msg->header;
                    
                    Intrinsic intrinsic;
                    intrinsic.fx = scaled_fx;
                    intrinsic.fy = scaled_fy;
                    intrinsic.cx = scaled_cx;
                    intrinsic.cy = scaled_cy;
                    
                    // Launch async point cloud generation
                    pointcloud_future_ = std::async(std::launch::async, 
                        &StereoMatchingNode::generateAndPublishPointCloudAsync,
                        this, rgb_clone, depth_clone, intrinsic, header_copy);
                }
            }
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in callback: %s", e.what());
        }
    }
    
    // Configuration
    std::unique_ptr<seek_disparity::ConfigManager> config_manager_;
    
    // TensorRT inference
    std::unique_ptr<TrtInferCore> infer_engine_;
    
    // Camera parameters
    float fx_, fy_, cx_, cy_, baseline_;
    float min_depth_, max_depth_;
    
    // Inference parameters
    int inference_width_, inference_height_;
    
    // Depth processing parameters
    bool use_confidence_filter_;
    float filter_percentile_;
    bool use_gradient_filter_;
    float gradient_threshold_;
    int gradient_dilation_size_;
    
    // Point cloud parameters
    bool enable_pointcloud_;
    
    // Inference control
    bool enable_inference_ = true;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_inference_;
    
    // Async point cloud generation
    std::future<void> pointcloud_future_;
    
    void enable_inference_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        enable_inference_ = request->data;
        response->success = true;
        response->message = enable_inference_ ? "Inference enabled" : "Inference disabled";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    
    void generateAndPublishPointCloudAsync(
        cv::Mat rgb, cv::Mat depth, Intrinsic intrinsic, std_msgs::msg::Header header) {
        try {
            auto pc_start = std::chrono::high_resolution_clock::now();
            
            if (!pointcloud_generator_->initialize(depth.rows, depth.cols, 1.0f)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize point cloud generator");
                return;
            }
            
            PointCloudT::Ptr cloud(new PointCloudT());
            
            if (pointcloud_generator_->generatePointCloudFromDepth(rgb, depth, intrinsic, cloud)) {
                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(*cloud, cloud_msg);
                cloud_msg.header = header;
                pointcloud_pub_->publish(cloud_msg);
                
                auto pc_end = std::chrono::high_resolution_clock::now();
                double pc_ms = std::chrono::duration_cast<std::chrono::microseconds>(pc_end - pc_start).count() / 1000.0;
                
                RCLCPP_DEBUG(this->get_logger(), "PointCloud=%.2fms | Pts=%ld", pc_ms, cloud->size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to generate point cloud");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in async point cloud generation: %s", e.what());
        }
    }
    
    // Message filter types
    using ImageMsg = sensor_msgs::msg::Image;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    
    // Subscribers
    message_filters::Subscriber<ImageMsg> left_sub_;
    message_filters::Subscriber<ImageMsg> right_sub_;
    std::shared_ptr<Synchronizer> sync_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr confidence_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    
    // Point cloud generator
    std::unique_ptr<CUDAPointCloudGenerator> pointcloud_generator_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<StereoMatchingNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("stereo_matching"), "Node failed: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
