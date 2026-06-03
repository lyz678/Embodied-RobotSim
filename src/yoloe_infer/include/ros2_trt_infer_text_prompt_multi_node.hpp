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

namespace yoloe_infer {

class YoloeMultiTextPromptNode : public rclcpp::Node {
public:
    YoloeMultiTextPromptNode();
    ~YoloeMultiTextPromptNode() override = default;

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    void sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);

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
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    
    // Publisher for 3D detections with class ID and position
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_detections_3d_;
    
    // TF for coordinate transformation
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Ground filtering parameter (in odom frame)
    float ground_filter_z_min_;
    
    // Inference control
    bool enable_inference_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_inference_;
    void enable_inference_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

} // namespace yoloe_infer

#endif // ROS2_TRT_INFER_TEXT_PROMPT_MULTI_NODE_HPP