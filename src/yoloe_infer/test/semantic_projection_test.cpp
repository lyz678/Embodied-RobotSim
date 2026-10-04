#include "pointcloud_colorizer.hpp"
#include <cstring>
#include <iostream>
#include <limits>

using namespace yoloe_infer;
static void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
template<class T> static T field(const sensor_msgs::msg::PointCloud2& cloud, int i, int offset) {
    T value;
    std::memcpy(&value, cloud.data.data()+i*cloud.point_step+offset, sizeof(T));
    return value;
}
int main() {
    try {
        PointCloudColorizer colorizer({{1,{0,255,0}}, {2,{255,0,0}}}, {255,255,255});
        auto info = std::make_shared<sensor_msgs::msg::CameraInfo>();
        info->width=64; info->height=48;
        info->k={20,0,32,0,20,24,0,0,1};
        info->r={1,0,0,0,1,0,0,0,1};
        info->p={20,0,32,0,0,20,24,0,0,0,1,0};
        Detection sofa; sofa.bbox={31,23,3,3}; sofa.mask=cv::Mat::ones(3,3,CV_8UC1);
        sofa.conf=.7; sofa.class_id=1;
        pcl::PointCloud<pcl::PointXYZ> lidar, camera;
        camera.push_back({0,0,2});   // In mask, nearest surface.
        camera.push_back({0,0,3});   // Same pixel, occluded background.
        camera.push_back({0,0,-1});  // Behind camera.
        camera.push_back({10,0,1});  // Outside FOV.
        camera.push_back({.5,0,2});  // Inside FOV, outside mask.
        camera.push_back({std::numeric_limits<float>::quiet_NaN(),0,2});
        for (size_t i=0; i<camera.size(); ++i) lidar.push_back({float(i),2,3});
        std_msgs::msg::Header header; header.frame_id="mid360_link"; header.stamp.sec=42;
        auto cloud=colorizer.semantic_lidar_cloud(lidar,camera,info,{sofa},header,.15);
        require(cloud.width==lidar.size(), "Projection must preserve all finite lidar geometry");
        require(cloud.header.frame_id==header.frame_id && cloud.header.stamp.sec==42,
            "Raycast must retain lidar origin and scan timestamp");
        require(field<uint32_t>(cloud,0,12)==0x00ff00 && field<float>(cloud,0,16)==.7f,
            "Mask point receives class RGB and confidence");
        for (int i=1; i<6; ++i) require(field<uint32_t>(cloud,i,12)==0xffffff &&
            field<float>(cloud,i,16)==0, "Occluded/out-of-FOV/unmasked points remain unknown");
        for (int i=0; i<6; ++i) require(field<float>(cloud,i,0)==float(i) &&
            field<float>(cloud,i,4)==2 && field<float>(cloud,i,8)==3,
            "Camera projection cannot change original lidar endpoints");
        Detection cup=sofa; cup.class_id=2; cup.conf=.9;
        cloud=colorizer.semantic_lidar_cloud(lidar,camera,info,{sofa,cup},header,.15);
        require(field<uint32_t>(cloud,0,12)==0x0000ff && field<float>(cloud,0,16)==.9f,
            "Overlapping masks use higher detection confidence");
        sofa.mask=cv::Mat::zeros(3,3,CV_8UC1);
        cloud=colorizer.semantic_lidar_cloud(lidar,camera,info,{sofa},header,.15);
        require(field<uint32_t>(cloud,0,12)==0xffffff, "Bounding box alone must not color a point");
        // Retain the original depth source, including unknown geometric pixels.
        cv::Mat depth(info->height,info->width,CV_32F,cv::Scalar(2));
        sofa.mask=cv::Mat::ones(3,3,CV_8UC1);
        cloud=colorizer.semantic_cloud(depth,info,{sofa},header);
        require(cloud.width==64*48 && field<uint32_t>(cloud,24*64+32,12)==0x00ff00 &&
            field<uint32_t>(cloud,0,12)==0xffffff, "Depth source remains supported");
        std::cout << "PASS: lidar FOV/mask projection, occlusion, geometry, headers, and depth compatibility\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
