#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstring>
namespace yoloe_infer {
// PCL aligns PointXYZRGB to 32 bytes; ROS consumers need only the four fields.
// Pack exact XYZ and RGB bits into 16 bytes, keeping full grasp point density.
inline sensor_msgs::msg::PointCloud2 compact_xyzrgb(
 const pcl::PointCloud<pcl::PointXYZRGB>& cloud,const std_msgs::msg::Header& header){
 sensor_msgs::msg::PointCloud2 message;message.header=header;message.height=1;message.is_dense=cloud.is_dense;
 sensor_msgs::PointCloud2Modifier modifier(message);
 modifier.setPointCloud2Fields(4,"x",1,7,"y",1,7,"z",1,7,"rgb",1,7);modifier.resize(cloud.size());
 for(size_t i=0;i<cloud.size();++i){const auto& p=cloud[i];float values[4]={p.x,p.y,p.z,0};
  std::memcpy(values+3,&p.rgb,4);std::memcpy(message.data.data()+i*16,values,16);}
 return message;
}
}
