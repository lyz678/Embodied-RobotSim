#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "graspnet_ros/graspnet_cpp.hpp"
#include "graspnet_ros/trt_wrapper.hpp"
#include "graspnet_ros/cuda_utils.hpp"
#include "graspnet_ros/transform_utils.hpp"
#include "graspnet_ros/visualization_utils.hpp"
#include "graspnet_ros/grasp_processing.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/convex_hull.h>
#include <shape_msgs/msg/mesh.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace graspnet_ros;

class GraspNetNode : public rclcpp::Node {
public:
    GraspNetNode() : Node("graspnet_node") {
        // Declare all parameters without default values
        this->declare_parameter("engine_path", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("plugin_path", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("num_point", rclcpp::ParameterType::PARAMETER_INTEGER);
        this->declare_parameter("frame_id", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("yoloe_config_path", rclcpp::ParameterType::PARAMETER_STRING);
        
        // Topic parameters
        this->declare_parameter("topics.input_cloud", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("topics.detections_3d", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("topics.cloud_filter", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("topics.grasp_markers", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("topics.sampled_cloud", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("topics.grasp_pose", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("topics.matched_detections", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("topics.matched_markers", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("topics.grasp_axes", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("topics.convex_hull", rclcpp::ParameterType::PARAMETER_STRING);
        this->declare_parameter("topics.collision_object", rclcpp::ParameterType::PARAMETER_STRING);
        
        // Queue size parameters
        this->declare_parameter("queue_sizes.input_cloud", rclcpp::ParameterType::PARAMETER_INTEGER);
        this->declare_parameter("queue_sizes.detections_3d", rclcpp::ParameterType::PARAMETER_INTEGER);
        this->declare_parameter("queue_sizes.cloud_filter", rclcpp::ParameterType::PARAMETER_INTEGER);
        this->declare_parameter("queue_sizes.publishers", rclcpp::ParameterType::PARAMETER_INTEGER);
        
        // NMS parameters
        this->declare_parameter("nms.distance_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE);
        this->declare_parameter("nms.angle_threshold_deg", rclcpp::ParameterType::PARAMETER_DOUBLE);
        
        // Filtering parameters
        this->declare_parameter("filtering.white_point_threshold", rclcpp::ParameterType::PARAMETER_INTEGER);
        this->declare_parameter("filtering.color_tolerance", rclcpp::ParameterType::PARAMETER_INTEGER);

        // Get basic parameters
        std::string engine_path = this->get_parameter("engine_path").as_string();
        std::string plugin_path = this->get_parameter("plugin_path").as_string();
        num_point_ = this->get_parameter("num_point").as_int();
        target_frame_ = this->get_parameter("frame_id").as_string();
        yoloe_config_path_ = this->get_parameter("yoloe_config_path").as_string();
        
        // Get topic names
        std::string topic_input_cloud = this->get_parameter("topics.input_cloud").as_string();
        std::string topic_detections = this->get_parameter("topics.detections_3d").as_string();
        std::string topic_cloud_filter = this->get_parameter("topics.cloud_filter").as_string();
        std::string topic_grasp_markers = this->get_parameter("topics.grasp_markers").as_string();
        std::string topic_sampled_cloud = this->get_parameter("topics.sampled_cloud").as_string();
        std::string topic_grasp_pose = this->get_parameter("topics.grasp_pose").as_string();
        std::string topic_matched_detections = this->get_parameter("topics.matched_detections").as_string();
        std::string topic_matched_markers = this->get_parameter("topics.matched_markers").as_string();
        std::string topic_grasp_axes = this->get_parameter("topics.grasp_axes").as_string();
        std::string topic_convex_hull = this->get_parameter("topics.convex_hull").as_string();
        std::string topic_collision_object = this->get_parameter("topics.collision_object").as_string();
        
        // Get queue sizes
        int queue_input_cloud = this->get_parameter("queue_sizes.input_cloud").as_int();
        int queue_detections = this->get_parameter("queue_sizes.detections_3d").as_int();
        int queue_cloud_filter = this->get_parameter("queue_sizes.cloud_filter").as_int();
        int queue_publishers = this->get_parameter("queue_sizes.publishers").as_int();
        
        // Get NMS parameters
        nms_distance_threshold_ = this->get_parameter("nms.distance_threshold").as_double();
        nms_angle_threshold_deg_ = this->get_parameter("nms.angle_threshold_deg").as_double();
        
        // Get filtering parameters
        white_point_threshold_ = this->get_parameter("filtering.white_point_threshold").as_int();
        color_tolerance_ = this->get_parameter("filtering.color_tolerance").as_int();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        if (engine_path.empty() || plugin_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameters engine_path and plugin_path must be set");
            // Don't exit, just skip init? Or shutdown.
        } else {
            init_engine(engine_path, plugin_path);
        }

        // Subscribers
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_input_cloud, queue_input_cloud, 
            std::bind(&GraspNetNode::cloud_callback, this, std::placeholders::_1));

        // Publishers
        pub_grasps_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_grasp_markers, queue_publishers);
        pub_sampled_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_sampled_cloud, queue_publishers);
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseArray>(topic_grasp_pose, queue_publishers);
        pub_matched_detections_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(topic_matched_detections, queue_publishers);
        pub_matched_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_matched_markers, queue_publishers);
        pub_grasp_axes_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_grasp_axes, queue_publishers);

        // Subscriber for detections
        sub_detections_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
             topic_detections, queue_detections,
             std::bind(&GraspNetNode::detections_callback, this, std::placeholders::_1));
        
        // Subscriber for cloud filter
        sub_cloud_filter_ = this->create_subscription<std_msgs::msg::Int32>(
            topic_cloud_filter, queue_cloud_filter,
            std::bind(&GraspNetNode::cloud_filter_callback, this, std::placeholders::_1));
        
        // Publishers for convex hull and collision object
        pub_convex_hull_ = this->create_publisher<shape_msgs::msg::Mesh>(topic_convex_hull, queue_publishers);
        pub_collision_object_ = this->create_publisher<moveit_msgs::msg::CollisionObject>(topic_collision_object, queue_publishers);
        
        // Load color mapping from YOLOE config
        load_color_mapping();
        
        // Initialize inference control
        this->declare_parameter("enable_inference", true);
        enable_inference_ = this->get_parameter("enable_inference").as_bool();
        
        // Create service to control inference
        srv_enable_inference_ = this->create_service<std_srvs::srv::SetBool>(
            "~/enable_inference",
            std::bind(&GraspNetNode::enable_inference_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Inference control initialized (enabled=%s)", 
                    enable_inference_ ? "true" : "false");

        RCLCPP_INFO(this->get_logger(), "GraspNet Node Initialized");
    }

    ~GraspNetNode() {
        if (d_input_) cudaFree(d_input_);
        for (void* ptr : d_outputs_) cudaFree(ptr);
    }

private:
    void init_engine(const std::string& engine_path, const std::string& plugin_path) {
        RCLCPP_INFO(this->get_logger(), "Loading Engine: %s", engine_path.c_str());
        trt_net_ = std::make_unique<TrtWrapper>(engine_path, plugin_path);

        // Allocate Input Buffer
        checkCudaErrors(cudaMalloc(&d_input_, num_point_ * 3 * sizeof(float)));

        // Allocate Output Buffers
        std::vector<std::string> output_names = trt_net_->getOutputNames();
        for (const auto& name : output_names) {
            size_t size = 0;
            int N = num_point_;
            // Allocating conservatively as per demo_trt.cpp
             if (name == "grasp_score_pred" || name == "grasp_width_pred" || name == "grasp_tolerance_pred") {
                size = 12 * N * 4 * sizeof(float);
            } else if (name == "grasp_angle_cls_pred") {
                 size = 12 * N * 1 * sizeof(float);
            } else if (name == "objectness_score") {
                size = 2 * N * 1 * sizeof(float);
            } else if (name == "grasp_center" || name == "approaching" || name == "fp2_xyz" || name == "grasp_top_view_xyz") {
                size = N * 3 * sizeof(float);
            } else {
                size = N * 60 * sizeof(float); 
            }
            
            void* ptr;
            checkCudaErrors(cudaMalloc(&ptr, size));
            d_outputs_.push_back(ptr);
            output_map_[name] = ptr;
            host_outputs_[name].resize(size / sizeof(float));
        }
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!trt_net_) return;
        
        // Skip processing if inference is disabled
        if (!enable_inference_) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "Inference disabled, skipping point cloud");
            return;
        }

        // Convert ROS Cloud to PCL
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
        bool do_transform = false;

        // Transform to target frame if needed
        if (msg->header.frame_id != target_frame_) {
            try {
                // Wait for transform
                 if (!tf_buffer_->canTransform(target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.5))) {
                     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for transform %s -> %s", msg->header.frame_id.c_str(), target_frame_.c_str());
                     return;
                 }
                
                geometry_msgs::msg::TransformStamped transform_stamped = 
                    tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp);
                
                transform_matrix = tfToEigenMatrix(transform_stamped);
                do_transform = true;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
                return; 
            }
        }

        // Convert to GraspNet PointCloud struct
        PointCloud cloud;
        cloud.points.reserve(pcl_cloud.size());
        cloud.colors.reserve(pcl_cloud.size());
        
        // Color-filtered cloud for convex hull
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        bool has_target_filter = (target_class_id_ >= 0 && class_to_color_.find(target_class_id_) != class_to_color_.end());
        
        if (has_target_filter) {
            cv::Vec3b target_color = class_to_color_[target_class_id_];
            RCLCPP_INFO(this->get_logger(), "Filtering for class %d with color BGR(%d,%d,%d)", 
                       target_class_id_, target_color[0], target_color[1], target_color[2]);
        }

        for (const auto& pt : pcl_cloud) {
            // Check NaN
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            
            // Filter white points (background)
            if (pt.r > white_point_threshold_ && pt.g > white_point_threshold_ && pt.b > white_point_threshold_) {
                continue;
            }
            
            // Check if point matches target color
            if (has_target_filter) {
                cv::Vec3b target_color = class_to_color_[target_class_id_];
                cv::Vec3b point_color(pt.b, pt.g, pt.r);  // PCL is RGB, cv::Vec3b is BGR
                
                // Color matching with tolerance
                if (std::abs(point_color[0] - target_color[0]) < color_tolerance_ &&
                    std::abs(point_color[1] - target_color[1]) < color_tolerance_ &&
                    std::abs(point_color[2] - target_color[2]) < color_tolerance_) {
                    color_filtered_cloud->push_back(pt);
                }
            }

            cloud.points.emplace_back(pt.x, pt.y, pt.z);
            cloud.colors.emplace_back(pt.r / 255.0f, pt.g / 255.0f, pt.b / 255.0f);
        }

        if (cloud.points.empty()) return;
        
        // Store filtered cloud for convex hull computation (transform to odom frame)
        if (has_target_filter && !color_filtered_cloud->empty()) {
            // Transform filtered cloud to target frame (odom) for convex hull
            if (do_transform) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::transformPointCloud(*color_filtered_cloud, *transformed_cloud, transform_matrix);
                filtered_cloud_ = transformed_cloud;
                RCLCPP_INFO(this->get_logger(), "Filtered and transformed %zu points to %s frame for class %d", 
                           filtered_cloud_->size(), target_frame_.c_str(), target_class_id_);
            } else {
                filtered_cloud_ = color_filtered_cloud;
                RCLCPP_INFO(this->get_logger(), "Filtered %zu points (already in %s) for class %d", 
                           filtered_cloud_->size(), target_frame_.c_str(), target_class_id_);
            }
        } else if (has_target_filter) {
            RCLCPP_WARN(this->get_logger(), "No points found matching target color for class %d", target_class_id_);
        }

        // Sample
        std::vector<Eigen::Vector3f> sampled_points;
        std::vector<Eigen::Vector3f> sampled_colors;
        sample_points(cloud, num_point_, sampled_points, sampled_colors);

        // Upload to GPU
        checkCudaErrors(cudaMemcpy(d_input_, sampled_points.data(), num_point_ * 3 * sizeof(float), cudaMemcpyHostToDevice));

        // Inference
        RCLCPP_INFO(this->get_logger(), "Inference...");
        trt_net_->infer(d_input_, num_point_, d_outputs_);

        // Copy back
        int i = 0;
        std::vector<std::string> output_names = trt_net_->getOutputNames();
        for (const auto& name : output_names) {
            checkCudaErrors(cudaMemcpy(host_outputs_[name].data(), d_outputs_[i], host_outputs_[name].size() * sizeof(float), cudaMemcpyDeviceToHost));
            i++;
        }

        // Decode
        nvinfer1::Dims obj_shape = trt_net_->getTensorShape("objectness_score");
        int N_out = obj_shape.d[2]; // assuming [1, 2, N] or similar
        std::vector<Grasp> grasps = pred_decode(host_outputs_, 1, N_out);

         // NMS
        float nms_th_angle = nms_angle_threshold_deg_ / 180.0f * M_PI;
        nms_grasps(grasps, nms_distance_threshold_, nms_th_angle);

        RCLCPP_INFO(this->get_logger(), "Found %zu grasps", grasps.size());

        // Publish
        // Transform sampled points to target frame for Visualization
        std::vector<Eigen::Vector3f> sampled_points_vis = sampled_points;
        if (do_transform) {
             transformPoints(sampled_points_vis, transform_matrix);
        }

        publish_sampled_cloud(sampled_points_vis, sampled_colors, msg->header);
        publish_markers(grasps, msg->header);

        // Match with detections
        match_and_publish_detections(grasps, msg->header.frame_id, msg->header.stamp);
        
        // Compute and publish convex hull if we have filtered cloud
        if (has_target_filter && filtered_cloud_ && !filtered_cloud_->empty()) {
            compute_and_publish_convex_hull(msg->header.stamp);
        }
    }
    
    void detections_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        latest_detections_ = *msg;
    }
    
    void cloud_filter_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        target_class_id_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Set target class filter to: %d", target_class_id_);
    }
    
    void load_color_mapping() {
        std::string config_path = yoloe_config_path_;
        
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            
            if (config["classes"]) {
                for (const auto& cls : config["classes"]) {
                    int id = cls["id"].as<int>();
                    std::vector<int> color_vec = cls["color"].as<std::vector<int>>();
                    // OpenCV uses BGR, config is RGB
                    cv::Vec3b color_bgr(color_vec[2], color_vec[1], color_vec[0]);
                    class_to_color_[id] = color_bgr;
                }
                RCLCPP_INFO(this->get_logger(), "Loaded %zu class color mappings", class_to_color_.size());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load color mapping: %s", e.what());
        }
    }
    
    void compute_and_publish_convex_hull(const rclcpp::Time& stamp) {
        if (!filtered_cloud_ || filtered_cloud_->empty()) {
            RCLCPP_WARN(this->get_logger(), "No filtered cloud to compute convex hull");
            return;
        }
        
        // Convert to XYZ cloud (convex hull doesn't need color)
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*filtered_cloud_, *xyz_cloud);
        
        // Compute convex hull
        pcl::ConvexHull<pcl::PointXYZ> hull;
        pcl::PointCloud<pcl::PointXYZ> hull_points;
        std::vector<pcl::Vertices> polygons;
        
        hull.setInputCloud(xyz_cloud);
        hull.setDimension(3);
        hull.setComputeAreaVolume(true);
        hull.reconstruct(hull_points, polygons);
        
        // Use original convex hull without inflation
        
        double volume = hull.getTotalVolume();
        double area = hull.getTotalArea();
        
        RCLCPP_INFO(this->get_logger(), "Convex hull: %zu vertices, %zu polygons, volume=%.4f, area=%.4f", 
                   hull_points.size(), polygons.size(), volume, area);
        
        if (hull_points.empty() || polygons.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to compute convex hull");
            return;
        }
        
        // Convert to Mesh message
        shape_msgs::msg::Mesh mesh_msg;
        
        // Add vertices
        for (const auto& pt : hull_points) {
            geometry_msgs::msg::Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            mesh_msg.vertices.push_back(p);
        }
        
        // Add triangles
        for (const auto& poly : polygons) {
            shape_msgs::msg::MeshTriangle triangle;
            if (poly.vertices.size() >= 3) {
                triangle.vertex_indices[0] = poly.vertices[0];
                triangle.vertex_indices[1] = poly.vertices[1];
                triangle.vertex_indices[2] = poly.vertices[2];
                mesh_msg.triangles.push_back(triangle);
            }
        }
        
        // Publish mesh
        pub_convex_hull_->publish(mesh_msg);
        
        // Create and publish CollisionObject
        moveit_msgs::msg::CollisionObject collision_obj;
        collision_obj.header.frame_id = target_frame_;
        collision_obj.header.stamp = stamp;
        collision_obj.id = "target_object_" + std::to_string(target_class_id_);
        
        // Add mesh to collision object
        collision_obj.meshes.push_back(mesh_msg);
        
        // Set pose (identity since mesh is already in target frame)
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        collision_obj.mesh_poses.push_back(pose);
        
        // Set operation to ADD
        collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
        
        pub_collision_object_->publish(collision_obj);
        
        RCLCPP_INFO(this->get_logger(), "Published convex hull and collision object for class %d", target_class_id_);
    }

    void match_and_publish_detections(const std::vector<Grasp>& grasps, const std::string& grasp_frame, const rclcpp::Time& stamp) {
        if (grasps.empty()) return;
        
        vision_msgs::msg::Detection3DArray matched_msg;
        
        // Use latest detections
        {
             std::lock_guard<std::mutex> lock(detection_mutex_);
             if (latest_detections_.detections.empty()) return;
             // Check time diff? For now just use latest.
             matched_msg = latest_detections_;
        }
        
        // Prepare to publish updated detections
        vision_msgs::msg::Detection3DArray output_msg;
        output_msg.header = matched_msg.header; // Should be odom or whatever YOLOE publishes
        
        std::vector<Grasp> matched_grasps_vis;

        // Transform ALL grasps to detection frame (e.g., odom)
        std::string detection_frame = matched_msg.header.frame_id;
        
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            if (!tf_buffer_->canTransform(detection_frame, grasp_frame, stamp, rclcpp::Duration::from_seconds(0.5))) {
                 RCLCPP_WARN(this->get_logger(), "Cannot transform from %s to %s", grasp_frame.c_str(), detection_frame.c_str());
                 return;
            }
            transform_stamped = tf_buffer_->lookupTransform(detection_frame, grasp_frame, stamp);
        } catch (...) {
             return;
        }
        
        // Transform grasps and apply orientation correction
        Eigen::Matrix4f transform_matrix = tfToEigenMatrix(transform_stamped);
        std::vector<Grasp> grasps_in_det_frame = transformGrasps(grasps, transform_matrix);
        correctGraspOrientations(grasps_in_det_frame);
        
        // Match grasps to detections
        matched_grasps_vis = matchGraspsToDetections(grasps_in_det_frame, matched_msg);
        
        // Update detection poses with matched grasps
        updateDetectionPoses(matched_msg, matched_grasps_vis);
        
        // Copy only successfully matched detections to output
        for (size_t i = 0; i < matched_msg.detections.size(); ++i) {
            if (matched_grasps_vis[i].score >= 0) {  // Valid match
                output_msg.detections.push_back(matched_msg.detections[i]);
            }
        }
        
        pub_matched_detections_->publish(output_msg);
        
        // Publish matched grasp poses as PoseArray on /grasp_pose
        if (!matched_grasps_vis.empty()) {
             std_msgs::msg::Header h = matched_msg.header; // has frame_id
             publish_best_grasp_pose(matched_grasps_vis, h);
             
             // Also publish markers for visual validation
             publish_markers(matched_grasps_vis, h, true);
             publish_grasp_axes(matched_grasps_vis, h);
        }
    }

    void publish_best_grasp_pose(const std::vector<Grasp>& matched_grasps, const std_msgs::msg::Header& header) {
        if (matched_grasps.empty()) return;
        auto pose_array = createPoseArray(matched_grasps, header);
        pub_pose_->publish(pose_array);
    }

    void publish_grasp_axes(const std::vector<Grasp>& grasps, const std_msgs::msg::Header& header) {
        auto markers = createGraspAxesMarkers(grasps, header);
        pub_grasp_axes_->publish(markers);
    }

    void publish_sampled_cloud(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& colors, const std_msgs::msg::Header& header) {
        auto msg = createPointCloudMsg(points, colors, header, target_frame_);
        pub_sampled_->publish(msg);
    }

    void publish_markers(const std::vector<Grasp>& grasps, const std_msgs::msg::Header& header, bool is_matched = false) {
        std::string ns = is_matched ? "matched_grasps" : "grasps";
        auto markers = createGraspMarkers(grasps, header, ns, is_matched, 10);
        
        if (is_matched)
             pub_matched_markers_->publish(markers);
        else
             pub_grasps_->publish(markers);
    }

    std::unique_ptr<TrtWrapper> trt_net_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub_detections_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_cloud_filter_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_grasps_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_matched_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_grasp_axes_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sampled_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_matched_detections_;
    rclcpp::Publisher<shape_msgs::msg::Mesh>::SharedPtr pub_convex_hull_;
    rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr pub_collision_object_;
    
    vision_msgs::msg::Detection3DArray latest_detections_;
    std::mutex detection_mutex_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // GPU buffers
    void* d_input_ = nullptr;
    std::vector<void*> d_outputs_;
    std::map<std::string, void*> output_map_;
    std::map<std::string, std::vector<float>> host_outputs_;
    
    int num_point_;
    std::string target_frame_;
    std::string yoloe_config_path_;
    
    // NMS parameters
    double nms_distance_threshold_;
    double nms_angle_threshold_deg_;
    
    // Filtering parameters
    int white_point_threshold_;
    int color_tolerance_;
    
    // Color filtering
    int target_class_id_ = -1;
    std::map<int, cv::Vec3b> class_to_color_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_;
    
    // Inference control
    bool enable_inference_ = true;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_inference_;
    
    void enable_inference_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        enable_inference_ = req->data;
        res->success = true;
        res->message = enable_inference_ ? "Inference enabled" : "Inference disabled";
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraspNetNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
