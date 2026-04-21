#ifndef POINTCLOUD_COLORIZER_HPP
#define POINTCLOUD_COLORIZER_HPP

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "trt_engine.hpp"
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace yoloe_infer {

// Structure to hold 3D coordinate information
struct Object3DCoordinate {
    std::string class_name;
    int class_id;
    float x, y, z;  // Position in base_footprint frame
    float confidence;
};

class PointCloudColorizer {
public:
    PointCloudColorizer(const std::map<int, cv::Vec3b>& color_mapping, const cv::Vec3b& default_color, float mad_threshold = 3.0f, float min_depth = 0.0f);
    ~PointCloudColorizer() = default;

    void process(
        const cv::Mat& depth_image,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
        const std::vector<Detection>& detections,
        pcl::PointCloud<pcl::PointXYZRGB>& cloud);



    // Calculate 3D coordinates for detections
    std::vector<Object3DCoordinate> calculate_3d_coordinates(
        const std::vector<Detection>& detections,
        const cv::Mat& depth_image,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
        const std::string& source_frame,
        const rclcpp::Time& stamp,
        tf2_ros::Buffer& tf_buffer,
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock);

private:
    // Structure to hold filtered depth data for a detection
    struct FilteredDepthData {
        std::vector<float> depths;
        std::vector<int> pixel_xs;  // Pixel x coordinates
        std::vector<int> pixel_ys;  // Pixel y coordinates
        float median_depth;
        float mad;
        cv::Rect roi;
    };

    // Common function to extract and filter depth data from a detection mask
    bool extract_filtered_depth(
        const Detection& det,
        const cv::Mat& depth_image,
        FilteredDepthData& result);

    void process_detections(
        const std::vector<Detection>& detections,
        const cv::Mat& depth_image,
        cv::Mat& color_image,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);

    void generate_pointcloud(
        const cv::Mat& depth_image,
        const cv::Mat& color_image,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
        pcl::PointCloud<pcl::PointXYZRGB>& cloud);

    std::map<int, cv::Vec3b> color_mapping_;
    cv::Vec3b default_color_;
    float mad_threshold_;
    float min_depth_;
};

} // namespace yoloe_infer

#endif // POINTCLOUD_COLORIZER_HPP
