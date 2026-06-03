#include "graspnet_ros/visualization_utils.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <algorithm>

namespace graspnet_ros {

sensor_msgs::msg::PointCloud2 createPointCloudMsg(
    const std::vector<Eigen::Vector3f>& points,
    const std::vector<Eigen::Vector3f>& colors,
    const std_msgs::msg::Header& header,
    const std::string& target_frame
) {
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl_cloud.width = points.size();
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = true;
    
    for (size_t i = 0; i < points.size(); ++i) {
        pcl::PointXYZRGB p;
        p.x = points[i].x();
        p.y = points[i].y();
        p.z = points[i].z();
        
        p.r = static_cast<uint8_t>(std::min(std::max(colors[i].x() * 255.0f, 0.0f), 255.0f));
        p.g = static_cast<uint8_t>(std::min(std::max(colors[i].y() * 255.0f, 0.0f), 255.0f));
        p.b = static_cast<uint8_t>(std::min(std::max(colors[i].z() * 255.0f, 0.0f), 255.0f));
        pcl_cloud.push_back(p);
    }
    
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pcl_cloud, msg);
    msg.header.stamp = header.stamp;
    msg.header.frame_id = target_frame;
    
    return msg;
}

visualization_msgs::msg::MarkerArray createGraspMarkers(
    const std::vector<Grasp>& grasps,
    const std_msgs::msg::Header& header,
    const std::string& namespace_prefix,
    bool is_matched,
    int max_count
) {
    visualization_msgs::msg::MarkerArray markers;
    int limit = std::min(static_cast<int>(grasps.size()), max_count);
    
    for (int i = 0; i < limit; ++i) {
        const auto& g = grasps[i];
        
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.ns = namespace_prefix;
        m.id = i;
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.003; // Line width
        
        // Score color (Red=high, Blue=low) or Green for matched
        if (is_matched) {
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
        } else {
            float s = std::min(std::max(g.score, 0.0f), 2.0f) / 2.0f;
            m.color.a = 1.0;
            m.color.r = s;
            m.color.b = 1.0 - s;
            m.color.g = 0.0;
        }
        
        // Gripper geometry
        float w = g.width;
        float d = g.depth;
        float finger_width = 0.004f;
        float depth_base = 0.02f;
        
        // Helper to transform local points to world
        auto transform = [&](float x, float y, float z) {
            Eigen::Vector3f local(x, y, z);
            Eigen::Vector3f world = g.rotation * local + g.translation;
            geometry_msgs::msg::Point p;
            p.x = world.x(); p.y = world.y(); p.z = world.z();
            return p;
        };
        
        // Base line
        m.points.push_back(transform(-depth_base, -w/2 - finger_width, 0));
        m.points.push_back(transform(-depth_base, w/2 + finger_width, 0));
        
        // Left finger
        m.points.push_back(transform(-depth_base, -w/2 - finger_width, 0));
        m.points.push_back(transform(d, -w/2 - finger_width, 0));
        
        // Right finger
        m.points.push_back(transform(-depth_base, w/2 + finger_width, 0));
        m.points.push_back(transform(d, w/2 + finger_width, 0));
        
        markers.markers.push_back(m);
    }
    
    return markers;
}

visualization_msgs::msg::MarkerArray createGraspAxesMarkers(
    const std::vector<Grasp>& grasps,
    const std_msgs::msg::Header& header,
    float axis_length,
    float line_width
) {
    visualization_msgs::msg::MarkerArray markers;
    
    for (size_t i = 0; i < grasps.size(); ++i) {
        const auto& g = grasps[i];
        
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.ns = "grasp_axes";
        m.id = i;
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = line_width;
        m.pose.orientation.w = 1.0; // Identity
        
        // Calculate axis endpoints
        Eigen::Vector3f origin = g.translation;
        Eigen::Vector3f x_axis = g.rotation.col(0) * axis_length + origin;
        Eigen::Vector3f y_axis = g.rotation.col(1) * axis_length + origin;
        Eigen::Vector3f z_axis = g.rotation.col(2) * axis_length + origin;
        
        geometry_msgs::msg::Point p_origin, p_x, p_y, p_z;
        p_origin.x = origin.x(); p_origin.y = origin.y(); p_origin.z = origin.z();
        p_x.x = x_axis.x(); p_x.y = x_axis.y(); p_x.z = x_axis.z();
        p_y.x = y_axis.x(); p_y.y = y_axis.y(); p_y.z = y_axis.z();
        p_z.x = z_axis.x(); p_z.y = z_axis.y(); p_z.z = z_axis.z();
        
        // X Line (Red)
        m.points.push_back(p_origin); m.points.push_back(p_x);
        std_msgs::msg::ColorRGBA c_red; c_red.r = 1.0; c_red.a = 1.0;
        m.colors.push_back(c_red); m.colors.push_back(c_red);
        
        // Y Line (Green)
        m.points.push_back(p_origin); m.points.push_back(p_y);
        std_msgs::msg::ColorRGBA c_green; c_green.g = 1.0; c_green.a = 1.0;
        m.colors.push_back(c_green); m.colors.push_back(c_green);
        
        // Z Line (Blue)
        m.points.push_back(p_origin); m.points.push_back(p_z);
        std_msgs::msg::ColorRGBA c_blue; c_blue.b = 1.0; c_blue.a = 1.0;
        m.colors.push_back(c_blue); m.colors.push_back(c_blue);
        
        markers.markers.push_back(m);
    }
    
    return markers;
}

geometry_msgs::msg::PoseArray createPoseArray(
    const std::vector<Grasp>& grasps,
    const std_msgs::msg::Header& header
) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = header;
    
    for (const auto& g : grasps) {
        geometry_msgs::msg::Pose p;
        p.position.x = g.translation.x();
        p.position.y = g.translation.y();
        p.position.z = g.translation.z();
        
        Eigen::Quaternionf q(g.rotation);
        p.orientation.w = q.w();
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        
        pose_array.poses.push_back(p);
    }
    
    return pose_array;
}

} // namespace graspnet_ros
