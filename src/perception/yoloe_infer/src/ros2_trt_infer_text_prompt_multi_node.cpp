#include "ros2_trt_infer_text_prompt_multi_node.hpp"
#include "compact_cloud.hpp"
#include <filesystem>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

namespace yoloe_infer {

YoloeMultiTextPromptNode::YoloeMultiTextPromptNode()
    : Node("yoloe_multi_text_prompt") {

    // Declare and get config path parameter
    this->declare_parameter("config_path", rclcpp::ParameterType::PARAMETER_STRING);
    std::string config_path = this->get_parameter("config_path").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Loading configuration from: %s", config_path.c_str());
    YAML::Node config = YAML::LoadFile(config_path);
    
    // Parse classes
    std::vector<std::string> prompts;
    colors_.clear();
    
    std::map<int, cv::Vec3b> color_map;
    const auto& classes = config["classes"];
    for (const auto& cls : classes) {
        std::string name = cls["name"].as<std::string>();
        int id = cls["id"].as<int>();
        std::vector<int> color_vec = cls["color"].as<std::vector<int>>();
        cv::Vec3b color_bgr(color_vec[2], color_vec[1], color_vec[0]); // Config is RGB, OpenCV is BGR
        prompts.push_back(name);
        colors_.push_back(color_bgr);
        color_map[id] = color_bgr;
    }
    
    // Default color from config or gray
    std::vector<int> rgb = config["default_color"].as<std::vector<int>>();
    cv::Vec3b default_color = cv::Vec3b(rgb[2], rgb[1], rgb[0]);
    
    // Parameters from config
    std::string engine_path = config["engine_path"].as<std::string>("");
    if (engine_path.rfind("package://", 0) == 0) {
        const auto slash = engine_path.find('/', 10);
        engine_path = ament_index_cpp::get_package_share_directory(engine_path.substr(10, slash - 10)) + engine_path.substr(slash);
    }

    float conf = config["conf_thres"].as<float>();
    float iou = config["iou_thres"].as<float>();
    float mad_threshold = config["mad_threshold"].as<float>();
    bool use_mad_filter = config["use_mad_filter"].as<bool>(true);
    std::string image_topic = config["image_topic"].as<std::string>();
    std::string depth_topic = this->declare_parameter<std::string>("depth_topic", config["depth_topic"].as<std::string>());
    std::string info_topic = config["camera_info_topic"].as<std::string>();
    int num_classes = config["num_classes"].as<int>();
    float min_depth = config["min_depth"].as<float>(0.0f);
    
    // Output topics
    std::string image_result_topic = config["image_result_topic"].as<std::string>();
    std::string pointcloud_colored_topic = config["pointcloud_colored_topic"].as<std::string>();
    std::string detections_3d_topic = config["detections_3d_topic"].as<std::string>();
    
    // Ground filtering parameter
    ground_filter_z_min_ = config["ground_filter_z_min"].as<float>(0.0f);

    RCLCPP_INFO(this->get_logger(), "Topics Configuration:");
    RCLCPP_INFO(this->get_logger(), "  - Image: %s", image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Depth: %s", depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Info: %s", info_topic.c_str());

    // Load tokenizer
    tokenizer_ = std::make_shared<SimpleTokenizer>();

    // Load engine
    engine_ = std::make_unique<MultiTextPromptTRTEngine>(engine_path, tokenizer_, num_classes, conf, iou);

    // Initialize PointCloudColorizer
    pointcloud_colorizer_ = std::make_unique<PointCloudColorizer>(color_map, default_color, use_mad_filter, mad_threshold, min_depth);
    
    // Set prompts from config
    engine_->set_prompts(prompts);
    engine_->warmup();

    // Synchronized subscribers
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    sub_image_sync_.subscribe(this, image_topic, qos.get_rmw_qos_profile());
    sub_info_sync_.subscribe(this, info_topic, qos.get_rmw_qos_profile());

    auto readonly = rcl_interfaces::msg::ParameterDescriptor{};
    readonly.read_only = true;
    semantic_depth_stride_ = declare_parameter<int>("semantic_depth_stride", config["semantic_depth_stride"].as<int>(1), readonly);
    if (semantic_depth_stride_ < 1 || semantic_depth_stride_ > 8) throw std::runtime_error("semantic_depth_stride must be in [1,8]");
    semantic_cloud_source_ = declare_parameter<std::string>("semantic_cloud_source", config["semantic_cloud_source"].as<std::string>("depth"), readonly);
    auto lidar_topic = declare_parameter<std::string>("semantic_lidar_topic", config["semantic_lidar_topic"].as<std::string>("/fastlio/cloud_odom"), readonly);
    lidar_frame_ = declare_parameter<std::string>("semantic_lidar_frame", config["semantic_lidar_frame"].as<std::string>("mid360_link"), readonly);
    projection_fixed_frame_ = declare_parameter<std::string>("projection_fixed_frame", config["projection_fixed_frame"].as<std::string>("odom"), readonly);
    lidar_sync_slop_ = declare_parameter<double>("projection_sync_slop", config["projection_sync_slop"].as<double>(0.15), readonly);
    occlusion_tolerance_ = declare_parameter<double>("projection_occlusion_tolerance", config["projection_occlusion_tolerance"].as<double>(0.15), readonly);
    if (!std::isfinite(lidar_sync_slop_) || lidar_sync_slop_ <= 0 ||
        !std::isfinite(occlusion_tolerance_) || occlusion_tolerance_ < 0 ||
        lidar_frame_.empty() || projection_fixed_frame_.empty()) {
        throw std::runtime_error("Invalid semantic lidar projection configuration");
    }
    if (semantic_cloud_source_ == "fastlio") {
        sub_lidar_sync_.subscribe(this, lidar_topic, qos.get_rmw_qos_profile());
        lidar_sync_ = std::make_shared<message_filters::Synchronizer<LidarSyncPolicy>>(
            LidarSyncPolicy(5), sub_image_sync_, sub_lidar_sync_, sub_info_sync_);
        lidar_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(lidar_sync_slop_));
        lidar_sync_->registerCallback(std::bind(&YoloeMultiTextPromptNode::lidar_sync_callback,
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        RCLCPP_INFO(get_logger(), "Semantic geometry: FAST-LIO %s, lidar frame=%s, sync<=%.3f s",
            lidar_topic.c_str(), lidar_frame_.c_str(), lidar_sync_slop_);
    } else if (semantic_cloud_source_ == "depth") {
        sub_depth_sync_.subscribe(this, depth_topic, qos.get_rmw_qos_profile());
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_image_sync_, sub_depth_sync_, sub_info_sync_);
        sync_->registerCallback(std::bind(&YoloeMultiTextPromptNode::sync_callback,
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    } else {
        throw std::runtime_error("semantic_cloud_source must be fastlio or depth");
    }

    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>(image_result_topic, 10);
    // Preview must survive suspended inference and missing depth/CameraInfo
    // (for example, when grasping pauses the stereo depth node).
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS().keep_last(2),
        std::bind(&YoloeMultiTextPromptNode::image_callback, this, std::placeholders::_1));
    pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_colored_topic, 10);
    pub_semantic_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/pointcloud_semantic", rclcpp::SensorDataQoS().keep_last(2));
    pub_detections_3d_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(detections_3d_topic, 10);

    // Initialize TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize timing
    fps_start_time_ = this->now();
    last_process_time_ = this->now();
    
    // Initialize inference control
    this->declare_parameter("enable_inference", true);
    enable_inference_ = this->get_parameter("enable_inference").as_bool();
    this->declare_parameter("enable_pointcloud", true);
    enable_pointcloud_ = this->get_parameter("enable_pointcloud").as_bool();
    srv_enable_pointcloud_ = this->create_service<std_srvs::srv::SetBool>(
        "~/enable_pointcloud",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            enable_pointcloud_ = request->data;
            response->success = true;
            response->message = enable_pointcloud_ ? "Point cloud publishing enabled" :
                "Point cloud publishing disabled; inference and images remain active";
            RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        });

    
    // Create service to control inference
    srv_enable_inference_ = this->create_service<std_srvs::srv::SetBool>(
        "~/enable_inference",
        std::bind(&YoloeMultiTextPromptNode::enable_inference_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Inference control initialized (enabled=%s)", 
                enable_inference_ ? "true" : "false");
}

void YoloeMultiTextPromptNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
    if (enable_inference_) return; // Synchronized inference publishes annotations.
    try {
        auto image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
        pub_image_->publish(*image->toImageMsg());
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Live preview conversion failed: %s", e.what());
    }
}


std::vector<Detection> YoloeMultiTextPromptNode::infer_image(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

    // Run inference
    auto [detections, timings] = engine_->predict(cv_ptr->image);

    // Update FPS
    frame_count_++;
    rclcpp::Time now = this->now();
    double elapsed = (now - fps_start_time_).seconds();
    if (elapsed >= 1.0) {
        current_fps_ = frame_count_ / elapsed;
        frame_count_ = 0;
        fps_start_time_ = now;
    }

    // Publish annotated image
    cv::Mat annotated = draw_detections(cv_ptr->image, detections, timings);
    auto img_msg_out = cv_bridge::CvImage(image_msg->header, "bgr8", annotated).toImageMsg();
    pub_image_->publish(*img_msg_out);

    return detections;
}

void YoloeMultiTextPromptNode::lidar_sync_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
    try {
        if (!enable_pointcloud_) {
            if (enable_inference_) infer_image(image_msg);
            return;
        }
        if (image_msg->header.frame_id.empty() || cloud_msg->header.frame_id.empty() ||
            image_msg->header.frame_id != info_msg->header.frame_id ||
            image_msg->width != info_msg->width || image_msg->height != info_msg->height) {
            throw std::runtime_error("Image/CameraInfo frame or size mismatch");
        }
        const auto image_time = rclcpp::Time(image_msg->header.stamp);
        const auto cloud_time = rclcpp::Time(cloud_msg->header.stamp);
        if (std::abs((image_time-cloud_time).seconds()) > lidar_sync_slop_) {
            throw std::runtime_error("Image and single scan exceed projection_sync_slop");
        }
        // Motion-compensated projection: camera pose at IMAGE time, registered
        // cloud at SCAN time, both connected through the fixed odom frame.
        auto camera_tf = tf_buffer_->lookupTransform(image_msg->header.frame_id, image_time,
            cloud_msg->header.frame_id, cloud_time, projection_fixed_frame_,
            rclcpp::Duration::from_seconds(0.5));
        auto lidar_tf = tf_buffer_->lookupTransform(lidar_frame_, cloud_msg->header.frame_id,
            cloud_time, rclcpp::Duration::from_seconds(0.5));
        auto eigen_transform = [](const geometry_msgs::msg::TransformStamped& t) {
            Eigen::Quaternionf q(t.transform.rotation.w, t.transform.rotation.x,
                t.transform.rotation.y, t.transform.rotation.z);
            Eigen::Affine3f pose = Eigen::Affine3f::Identity();
            pose.linear() = q.normalized().toRotationMatrix();
            pose.translation() << t.transform.translation.x, t.transform.translation.y, t.transform.translation.z;
            return pose;
        };
        pcl::PointCloud<pcl::PointXYZ> original, camera_points, lidar_points;
        pcl::fromROSMsg(*cloud_msg, original);
        pcl::transformPointCloud(original, camera_points, eigen_transform(camera_tf));
        pcl::transformPointCloud(original, lidar_points, eigen_transform(lidar_tf));
        auto detections = enable_inference_ ? infer_image(image_msg) : std::vector<Detection>{};
        std_msgs::msg::Header header = cloud_msg->header;
        header.frame_id = lidar_frame_;
        pub_semantic_cloud_->publish(pointcloud_colorizer_->semantic_lidar_cloud(
            lidar_points, camera_points, info_msg, detections, header, occlusion_tolerance_));
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "FAST-LIO semantic scan rejected: %s", e.what());
    }
}

void YoloeMultiTextPromptNode::sync_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {

    // Suspend classification, not measured geometry. Raycasting still needs
    // background returns to clear voxels after an object has moved away.
    if (!enable_inference_) {
        if (!enable_pointcloud_) return;
        try {
            auto depth = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
            pub_semantic_cloud_->publish(pointcloud_colorizer_->semantic_cloud(
                depth->image, info_msg, {}, depth_msg->header, semantic_depth_stride_));
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Geometry-only depth frame rejected: %s", e.what());
        }
        return;
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received synchronized image, depth, and info!");
    static int sync_count = 0;
    if (++sync_count % 30 == 0) {
        RCLCPP_INFO(this->get_logger(), "Processing frame %d (FPS: %.2f)", sync_count, current_fps_);
    }

    const auto frame_begin = std::chrono::steady_clock::now();
    auto detections = infer_image(image_msg);
    const auto infer_done = std::chrono::steady_clock::now();

    // PointCloud Colorization
    cv_bridge::CvImagePtr depth_ptr;
    depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    // Publish independent geometry before legacy depth/mask processing. A bad
    // semantic detection must not remove a measured obstacle or shorten a ray.
    if (enable_pointcloud_) {
        pub_semantic_cloud_->publish(pointcloud_colorizer_->semantic_cloud(
            depth_ptr->image, info_msg, detections, depth_msg->header, semantic_depth_stride_));
    }
    const auto semantic_done = std::chrono::steady_clock::now();

    std::string source_frame = depth_msg->header.frame_id;            

    // Calculate 3D coordinates (using original depth and transforms inside this function, independent of the cloud above for now)
    // Note: calculate_3d_coordinates already does its own TF lookup internally.
    std::vector<Object3DCoordinate> coords = pointcloud_colorizer_->calculate_3d_coordinates(
        detections, depth_ptr->image, info_msg, source_frame, 
        depth_msg->header.stamp, *tf_buffer_, this->get_logger(), this->get_clock());
    
    // Publish 3D detections with class ID and position
    if (!coords.empty()) {
        vision_msgs::msg::Detection3DArray detections_3d_msg;
        detections_3d_msg.header.stamp = depth_msg->header.stamp;
        detections_3d_msg.header.frame_id = "odom"; // This is hardcoded to odom in calculate_3d_coordinates logic effectively
        
        for (const auto& coord : coords) {
            vision_msgs::msg::Detection3D det_3d;
            det_3d.header = detections_3d_msg.header;
            
            // Set position
            det_3d.bbox.center.position.x = coord.x;
            det_3d.bbox.center.position.y = coord.y;
            det_3d.bbox.center.position.z = coord.z;
            det_3d.bbox.center.orientation.w = 1.0;
            
            // Set class ID and confidence
            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = std::to_string(coord.class_id);
            hypothesis.hypothesis.score = coord.confidence;
            det_3d.results.push_back(hypothesis);
            
            detections_3d_msg.detections.push_back(det_3d);

            RCLCPP_DEBUG(this->get_logger(), "Object: %s (ID: %d), Confidence: %.2f, 3D Pose (odom): [x: %.3f, y: %.3f, z: %.3f]",
                        coord.class_name.c_str(), coord.class_id, coord.confidence, coord.x, coord.y, coord.z);
        }
        
        pub_detections_3d_->publish(detections_3d_msg);
    }
    if (!enable_pointcloud_) return; // Images and 3D detections still publish.
    const auto coordinates_done = std::chrono::steady_clock::now();

    // PointCloud Generation Part 2: FILTERED Point Cloud (for Octomap / Nav)
    // We generate this using the MODIFIED depth image
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
    pointcloud_colorizer_->process(depth_ptr->image, info_msg, detections, cloud_filtered);
    const auto legacy_done = std::chrono::steady_clock::now();

    // Transform to odom frame, apply ground filtering, then transform back to camera frame
    try {
        // Lookup transform from camera to odom
        geometry_msgs::msg::TransformStamped cam_to_odom = tf_buffer_->lookupTransform(
            "odom", depth_msg->header.frame_id, depth_msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
        
        // Ground filtering needs only odom Z. Keep the original camera points
        // instead of allocating and transforming the entire cloud twice.
        Eigen::Affine3d transform_cam_to_odom = Eigen::Affine3d::Identity();
        transform_cam_to_odom.translation() << cam_to_odom.transform.translation.x,
                                               cam_to_odom.transform.translation.y,
                                               cam_to_odom.transform.translation.z;
        Eigen::Quaterniond q_cam_to_odom(cam_to_odom.transform.rotation.w,
                                         cam_to_odom.transform.rotation.x,
                                         cam_to_odom.transform.rotation.y,
                                         cam_to_odom.transform.rotation.z);
        transform_cam_to_odom.rotate(q_cam_to_odom);
        pcl::PointCloud<pcl::PointXYZRGB> cloud_ground_filtered_cam;
        cloud_ground_filtered_cam.reserve(cloud_filtered.size());
        for (const auto& point : cloud_filtered) {
            const double z = transform_cam_to_odom.matrix()(2,0)*point.x +
                transform_cam_to_odom.matrix()(2,1)*point.y +
                transform_cam_to_odom.matrix()(2,2)*point.z + transform_cam_to_odom.translation().z();
            if (std::isfinite(z) && z >= ground_filter_z_min_ && z <= 100.0)
                cloud_ground_filtered_cam.push_back(point);
        }
        
        // Publish FILTERED Cloud in camera frame
        auto pc_msg_filtered = compact_xyzrgb(cloud_ground_filtered_cam, depth_msg->header);
        pub_pointcloud_->publish(pc_msg_filtered);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Could not transform point cloud to odom for filtering: %s", ex.what());
    }
    const auto finished = std::chrono::steady_clock::now();
    auto ms = [](auto a, auto b){return std::chrono::duration<double,std::milli>(b-a).count();};
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Frame ms: inference=%.1f semantic=%.1f coordinates=%.1f legacy=%.1f filter/publish=%.1f total=%.1f",
        ms(frame_begin,infer_done),ms(infer_done,semantic_done),ms(semantic_done,coordinates_done),
        ms(coordinates_done,legacy_done),ms(legacy_done,finished),ms(frame_begin,finished));
}

cv::Mat YoloeMultiTextPromptNode::draw_detections(
    const cv::Mat& image,
    const std::vector<Detection>& detections,
    const Timings& timings) {

    cv::Mat result = image.clone();

    for (const auto& det : detections) {
        // Convert center format to corner format
        int x1 = static_cast<int>(det.bbox.x);
        int y1 = static_cast<int>(det.bbox.y);
        int x2 = static_cast<int>(det.bbox.x + det.bbox.width);
        int y2 = static_cast<int>(det.bbox.y + det.bbox.height);

        // Choose color based on class ID
        cv::Vec3b color = colors_[det.class_id % colors_.size()];

        // Draw mask if available
        if (!det.mask.empty()) {
            cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
            
            // Clip ROI to image bounds
            roi = roi & cv::Rect(0, 0, result.cols, result.rows);
            
            if (roi.width > 0 && roi.height > 0) {
                 // Resize mask to match ROI if needed (should match if bbox logic is consistent)
                 cv::Mat mask_roi;
                 if (det.mask.size() != roi.size()) {
                     cv::resize(det.mask, mask_roi, roi.size(), 0, 0, cv::INTER_NEAREST);
                 } else {
                     mask_roi = det.mask;
                 }
                 
                 cv::Mat color_mask = cv::Mat::zeros(roi.size(), result.type());
                 color_mask.setTo(cv::Scalar(color[0], color[1], color[2]), mask_roi);
                 
                 cv::Mat result_roi = result(roi);
                 cv::addWeighted(result_roi, 1.0, color_mask, 0.4, 0.0, result_roi);
            }
        }

        // Draw rectangle
        cv::rectangle(result, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(color[0], color[1], color[2]), 2);

        // Draw label
        std::string label = det.name + " " + std::to_string(det.conf).substr(0, 4);
        cv::putText(result, label, cv::Point(x1, y1 - 5), cv::FONT_HERSHEY_SIMPLEX,
                    0.6, cv::Scalar(color[0], color[1], color[2]), 2);
    }

    // Draw info overlay
    // std::string stats = "FPS: " + std::to_string(current_fps_).substr(0, 4) +
    //                    " | Pre: " + std::to_string(timings.pre).substr(0, 4) +
    //                    " | Infer: " + std::to_string(timings.infer).substr(0, 4) +
    //                    " | Post: " + std::to_string(timings.post).substr(0, 4) + "ms";
    // cv::putText(result, stats, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
    //            0.6, cv::Scalar(0, 255, 0), 2);

    if (engine_) {
        const auto& prompts = engine_->get_current_prompts();
        if (!prompts.empty()) {
            std::string prompt_info = "Prompts: ";
            size_t max_display = std::min(size_t(3), prompts.size());
            for (size_t i = 0; i < max_display; ++i) {
                if (i > 0) prompt_info += ", ";
                prompt_info += prompts[i];
            }
            if (prompts.size() > 3) {
                prompt_info += " ...";
            }
            // cv::putText(result, prompt_info, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX,
            //            0.6, cv::Scalar(0, 255, 255), 2);
        }
    } else {
        cv::putText(result, "Engine not loaded", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX,
                   0.6, cv::Scalar(0, 0, 255), 2);
    }

    return result;
}

void YoloeMultiTextPromptNode::enable_inference_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    enable_inference_ = request->data;
    response->success = true;
    response->message = enable_inference_ ? "Inference enabled" : "Inference disabled; live image preview and geometry active";
    
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

} // namespace yoloe_infer

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<yoloe_infer::YoloeMultiTextPromptNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", e.what());
        return 1;
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown error occurred");
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
