#ifndef ROS2_TRT_INFER_TEXT_PROMPT_MULTI_NODE_HPP
#define ROS2_TRT_INFER_TEXT_PROMPT_MULTI_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>

#include "trt_engine.hpp"
#include "bot_sort.hpp"
#include "simple_tokenizer.hpp"
#include "pointcloud_colorizer.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <deque>
#include <chrono>

namespace yoloe_infer {

class YoloeMultiTextPromptNode : public rclcpp::Node {
public:
    YoloeMultiTextPromptNode();
    ~YoloeMultiTextPromptNode() override = default;

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    
    void sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);
    void process_depth(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);
    void drain_depth();
    struct PendingDepth {
        sensor_msgs::msg::Image::ConstSharedPtr image,depth;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
        std::chrono::steady_clock::time_point received;
    };
    std::deque<PendingDepth> pending_depth_;
    rclcpp::TimerBase::SharedPtr depth_timer_;

    void lidar_sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);
    std::vector<Detection> infer_image(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);

    void publish_annotated(const sensor_msgs::msg::Image::ConstSharedPtr& image,
        const std::vector<Detection>& detections);
    bool track_depth(std::vector<Detection>& detections,
        const sensor_msgs::msg::Image::ConstSharedPtr& image,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);
    void track_lidar(std::vector<Detection>& detections, const pcl::PointCloud<pcl::PointXYZ>& camera,
        const pcl::PointCloud<pcl::PointXYZ>& world, const sensor_msgs::msg::Image::ConstSharedPtr& image,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info, const Eigen::Matrix4d& world_to_camera);
    bool tracking_enabled_ = true;
    int tracking_min_hits_ = 2;
    float detection_confidence_ = .25f;
    void apply_tracking(std::vector<Detection>& detections, const std::vector<TrackObservation>& observations);
    std::unique_ptr<BotSort3D> tracker_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_tracks_;

    cv::Mat draw_detections(const cv::Mat& image,
                           const std::vector<Detection>& detections,
                           const Timings& timings);

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

    // Engine and tokenizer
    std::shared_ptr<SimpleTokenizer> tokenizer_;
    std::unique_ptr<MultiTextPromptTRTEngine> engine_;
    std::unique_ptr<PointCloudColorizer> pointcloud_colorizer_;

    // Bridge
    std::shared_ptr<cv_bridge::CvImage> bridge_;

    // Timing
    rclcpp::Time last_process_time_;
    double target_fps_ = 10.0;

    // FPS tracking
    int frame_count_ = 0;
    rclcpp::Time fps_start_time_;
    double current_fps_ = 0.0;

    // Colors for visualization
    std::vector<cv::Vec3b> colors_;
    
    // Sync
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_sync_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_depth_sync_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_info_sync_;
    
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo
    > SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar_sync_;
    using LidarSyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CameraInfo>;
    std::shared_ptr<message_filters::Synchronizer<LidarSyncPolicy>> lidar_sync_;
    bool publish_lidar_semantics_ = false;
    std::deque<std::pair<std::pair<int64_t,std::string>,std::vector<Detection>>> inference_cache_;
    std::string semantic_cloud_source_, lidar_frame_, projection_fixed_frame_;
    double lidar_sync_slop_, occlusion_tolerance_;
    int semantic_depth_stride_ = 1;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_semantic_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_semantic_cloud_;
    
    // Publisher for 3D detections with class ID and position
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_detections_3d_;
    
    // TF for coordinate transformation
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Ground filtering parameter (in odom frame)
    float ground_filter_z_min_;
    
    // Inference control
    bool enable_inference_;
    bool enable_pointcloud_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_pointcloud_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_inference_;
    void enable_inference_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

} // namespace yoloe_infer

#endif // ROS2_TRT_INFER_TEXT_PROMPT_MULTI_NODE_HPP
