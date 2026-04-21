#include "pointcloud_colorizer.hpp"
#include <image_geometry/pinhole_camera_model.hpp>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>

namespace yoloe_infer {

PointCloudColorizer::PointCloudColorizer(
    const std::map<int, cv::Vec3b>& color_mapping,
    const cv::Vec3b& default_color,
    float mad_threshold,
    float min_depth)
    : color_mapping_(color_mapping), default_color_(default_color), mad_threshold_(mad_threshold), min_depth_(min_depth) {
}



void PointCloudColorizer::process(
    const cv::Mat& depth_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
    const std::vector<Detection>& detections,
    pcl::PointCloud<pcl::PointXYZRGB>& cloud) {

    // 1. Prepare Color Map (Background = Default Color)
    cv::Mat color_image(depth_image.size(), CV_8UC3, default_color_);

    // 2. Process Masks and update Color Map
    process_detections(detections, depth_image, color_image, info_msg);

    // 3. Generate Point Cloud
    generate_pointcloud(depth_image, color_image, info_msg, cloud);
}

bool PointCloudColorizer::extract_filtered_depth(
    const Detection& det,
    const cv::Mat& depth_image,
    FilteredDepthData& result) {

    if (det.mask.empty()) {
        return false;
    }

    cv::Mat mask = det.mask;
    int w = mask.cols;
    int h = mask.rows;
    
    // Reconstruct ROI top-left
    int x1 = std::max(0, (int)det.bbox.x);
    int y1 = std::max(0, (int)det.bbox.y);
    
    // Basic bounds check
    if (x1 + w > depth_image.cols) w = depth_image.cols - x1;
    if (y1 + h > depth_image.rows) h = depth_image.rows - y1;
    if (w <= 0 || h <= 0) return false;

    result.roi = cv::Rect(x1, y1, w, h);
    cv::Mat depth_roi = depth_image(result.roi);
    
    // Use mask directly (it matches bbox)
    cv::Mat mask_roi = mask; 
    if (mask_roi.size() != result.roi.size()) {
        mask_roi = mask(cv::Rect(0, 0, w, h));
    }
    
    // Extract valid depth values from mask
    std::vector<float> valid_depths;
    result.pixel_xs.clear();
    result.pixel_ys.clear();
    valid_depths.reserve(w * h);
    result.pixel_xs.reserve(w * h);
    result.pixel_ys.reserve(w * h);
    
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            if (mask_roi.at<uint8_t>(r, c) > 0) {
                float d = 0.0f;
                if (depth_roi.type() == CV_16UC1) {
                    d = depth_roi.at<uint16_t>(r, c) * 0.001f;
                } else if (depth_roi.type() == CV_32FC1) {
                    d = depth_roi.at<float>(r, c);
                }
                
                if (std::isfinite(d) && d > 0.0f) {
                    valid_depths.push_back(d);
                    result.pixel_xs.push_back(x1 + c);
                    result.pixel_ys.push_back(y1 + r);
                }
            }
        }
    }

    if (valid_depths.empty()) {
        return false;
    }

    // Calculate Median
    std::vector<float> sorted_depths = valid_depths;
    std::sort(sorted_depths.begin(), sorted_depths.end());
    result.median_depth = sorted_depths[sorted_depths.size() / 2];

    // Calculate MAD
    std::vector<float> abs_devs;
    abs_devs.reserve(valid_depths.size());
    for (float d : valid_depths) {
        abs_devs.push_back(std::abs(d - result.median_depth));
    }
    std::sort(abs_devs.begin(), abs_devs.end());
    result.mad = abs_devs[abs_devs.size() / 2];

    // Filter outliers using MAD
    result.depths.clear();
    std::vector<int> filtered_xs, filtered_ys;
    result.depths.reserve(valid_depths.size());
    filtered_xs.reserve(valid_depths.size());
    filtered_ys.reserve(valid_depths.size());

    float k = 0.6745f;  // Constant for modified Z-score
    for (size_t i = 0; i < valid_depths.size(); ++i) {
        float modified_z = k * std::abs(valid_depths[i] - result.median_depth) / (result.mad + 1e-6f);
        if (modified_z < mad_threshold_) {
            result.depths.push_back(valid_depths[i]);
            filtered_xs.push_back(result.pixel_xs[i]);
            filtered_ys.push_back(result.pixel_ys[i]);
        }
    }

    // Update pixel coordinates to filtered ones
    result.pixel_xs = std::move(filtered_xs);
    result.pixel_ys = std::move(filtered_ys);

    return !result.depths.empty();
}

void PointCloudColorizer::process_detections(
    const std::vector<Detection>& detections,
    const cv::Mat& depth_image,
    cv::Mat& color_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {

    for (const auto& det : detections) {
        FilteredDepthData depth_data;
        if (!extract_filtered_depth(det, depth_image, depth_data)) {
            continue;
        }

        // Determine color
        cv::Vec3b cls_color = default_color_;
        if (color_mapping_.count(det.class_id)) {
            cls_color = color_mapping_[det.class_id];
        }

        // Apply color to inliers
        cv::Mat color_roi = color_image(depth_data.roi);
        cv::Mat depth_roi = depth_image(depth_data.roi);
        cv::Mat mask_roi = det.mask;
        if (mask_roi.size() != depth_data.roi.size()) {
            mask_roi = det.mask(cv::Rect(0, 0, depth_data.roi.width, depth_data.roi.height));
        }

        float k = 0.6745f / (depth_data.mad + 1e-6f);
        for (int r = 0; r < depth_data.roi.height; ++r) {
            for (int c = 0; c < depth_data.roi.width; ++c) {
                if (mask_roi.at<uint8_t>(r, c) > 0) {
                    float d = 0.0f;
                    if (depth_roi.type() == CV_16UC1) {
                        d = depth_roi.at<uint16_t>(r, c) * 0.001f;
                    } else if (depth_roi.type() == CV_32FC1) {
                        d = depth_roi.at<float>(r, c);
                    }
                    if (std::isfinite(d) && d > 0.0f) {
                        float modified_z = std::abs(d - depth_data.median_depth) * k;
                        if (modified_z < mad_threshold_) {
                            // Inlier - Color it
                            color_roi.at<cv::Vec3b>(r, c) = cls_color;
                        }
                    }
                }
            }
        }
    }
}
void PointCloudColorizer::generate_pointcloud(
    const cv::Mat& depth_image,
    const cv::Mat& color_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
    pcl::PointCloud<pcl::PointXYZRGB>& cloud) {

    // Initialize camera model
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(info_msg);

    // Pre-calculate validity mask using vectorized OpenCV operations
    cv::Mat valid_mask = (depth_image > min_depth_);

    // Find all valid non-zero pixels
    std::vector<cv::Point> valid_pixels;
    cv::findNonZero(valid_mask, valid_pixels);
    
    int num_valid = valid_pixels.size();
    cloud.points.resize(num_valid);
    cloud.width = num_valid;
    cloud.height = 1;
    cloud.is_dense = true;

    #pragma omp parallel for
    for (int i = 0; i < num_valid; ++i) {
        int u = valid_pixels[i].x;
        int v = valid_pixels[i].y;
        
        float d = depth_image.at<float>(v, u);
        pcl::PointXYZRGB& pt = cloud.points[i];
        
        // Use image_geometry projectPixelTo3dRay
        cv::Point3d ray = model.projectPixelTo3dRay(cv::Point2d(u, v));
        pt.x = ray.x * d;
        pt.y = ray.y * d;
        pt.z = ray.z * d;
        
        cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);
        pt.r = color[2]; // OpenCV is BGR
        pt.g = color[1];
        pt.b = color[0];
    }
}

std::vector<Object3DCoordinate> PointCloudColorizer::calculate_3d_coordinates(
    const std::vector<Detection>& detections,
    const cv::Mat& depth_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
    const std::string& source_frame,
    const rclcpp::Time& stamp,
    tf2_ros::Buffer& tf_buffer,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock) {

    std::vector<Object3DCoordinate> coordinates;

    if (detections.empty()) {
        return coordinates;
    }

    // Get camera intrinsics from CameraInfo
    // P matrix: [fx, 0, cx, Tx, 0, fy, cy, Ty, 0, 0, 1, 0]
    float fx = info_msg->p[0];
    float fy = info_msg->p[5];
    float cx = info_msg->p[2];
    float cy = info_msg->p[6];

    if (fx <= 0 || fy <= 0) {
        RCLCPP_WARN(logger, "Invalid camera intrinsics: fx=%.2f, fy=%.2f", fx, fy);
        return coordinates;
    }

    // Use source_frame directly as it now follows optical definition

    // Check if transform is available with timeout
    std::string err_str;
    bool use_latest = false;
    
    // Try to find the transform at the exact timestamp with a small timeout
    // This helps when camera data is slightly ahead of TF data
    bool has_transform = tf_buffer.canTransform(
        "odom", source_frame, stamp, 
        rclcpp::Duration::from_seconds(0.5), &err_str);

    if (!has_transform) {
        // Fallback: Try with latest available transform
        if (tf_buffer.canTransform("odom", source_frame, tf2::TimePointZero, &err_str)) {
             use_latest = true;
             RCLCPP_WARN_THROTTLE(logger, *clock, 5000, 
                "Transform to odom failed at specific timestamp (lag?), falling back to latest.");
        } else {
             RCLCPP_WARN_THROTTLE(logger, *clock, 5000, 
                "Cannot transform from %s to odom: %s", source_frame.c_str(), err_str.c_str());
             return coordinates;
        }
    }

    // Process each detection
    for (size_t det_idx = 0; det_idx < detections.size(); ++det_idx) {
        const auto& det = detections[det_idx];
        
        FilteredDepthData depth_data;
        if (!extract_filtered_depth(det, depth_image, depth_data)) {
            continue;
        }

        // Calculate 3D centroid in camera frame
        float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
        for (size_t i = 0; i < depth_data.depths.size(); ++i) {
            float d = depth_data.depths[i];
            float x_3d = (depth_data.pixel_xs[i] - cx) * d / fx;
            float y_3d = (depth_data.pixel_ys[i] - cy) * d / fy;
            float z_3d = d;
            sum_x += x_3d;
            sum_y += y_3d;
            sum_z += z_3d;
        }

        float x_cam = sum_x / depth_data.depths.size();
        float y_cam = sum_y / depth_data.depths.size();
        float z_cam = sum_z / depth_data.depths.size();

        // Transform point to odom
        geometry_msgs::msg::PointStamped point_camera;
        point_camera.header.frame_id = source_frame;
        point_camera.header.frame_id = source_frame;
        // Use TimePointZero if we are falling back to latest, otherwise use exact stamp
        point_camera.header.stamp = use_latest ? rclcpp::Time(0) : stamp;
        point_camera.point.x = x_cam;
        point_camera.point.y = y_cam;
        point_camera.point.z = z_cam;

        geometry_msgs::msg::PointStamped point_base;
        try {
             point_base = tf_buffer.transform(point_camera, "odom");
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(logger, "Transform failed for detection %zu: %s", det_idx, ex.what());
            continue;
        }

        // Create 3D coordinate result
        Object3DCoordinate coord;
        coord.class_name = det.name;
        coord.class_id = det.class_id;
        coord.x = point_base.point.x;
        coord.y = point_base.point.y;
        coord.z = point_base.point.z;
        coord.confidence = det.conf;

        coordinates.push_back(coord);
    }

    return coordinates;
}

} // namespace yoloe_infer
