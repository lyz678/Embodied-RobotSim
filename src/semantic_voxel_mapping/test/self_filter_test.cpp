#include "semantic_voxel_mapping/self_filter.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
int main(int argc,char** argv){
 rclcpp::init(argc,argv);
 try{
  RobotSelfFilter filter;
  sensor_msgs::msg::PointCloud2 cloud;cloud.header.frame_id="camera";cloud.height=1;cloud.is_dense=true;
  sensor_msgs::PointCloud2Modifier modify(cloud);modify.setPointCloud2FieldsByString(1,"xyz");modify.resize(3);
  sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
  for(float value:{1.f,1.51f,1.53f}){*x=value;*y=0;*z=0;++x;++y;++z;}
  tf2_ros::Buffer tf(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  bool rejected=false;try{filter.mask(cloud,tf);}catch(...){rejected=true;}
  if(!rejected)throw std::runtime_error("Missing description must reject self-filtered clouds");
  filter.load("<robot name='test'><link name='body'><collision><geometry><sphere radius='0.5'/></geometry></collision></link></robot>",.02);
  geometry_msgs::msg::TransformStamped transform;transform.header.frame_id="camera";transform.child_frame_id="body";
  transform.transform.translation.x=1;transform.transform.rotation.w=1;tf.setTransform(transform,"test",true);
  auto mask=filter.mask(cloud,tf);
  if(mask.size()!=3||mask[0]!=point_containment_filter::ShapeMask::INSIDE||
     mask[1]!=point_containment_filter::ShapeMask::INSIDE||mask[2]!=point_containment_filter::ShapeMask::OUTSIDE)
    throw std::runtime_error("Translated robot geometry/padding must remove self points and retain obstacles");
  cloud.header.frame_id="missing_camera";rejected=false;try{filter.mask(cloud,tf);}catch(...){rejected=true;}
  if(!rejected)throw std::runtime_error("Missing TF must reject the frame");
  std::cout<<"PASS: self filtering, padding, transformed obstacles and missing-input rejection\n";
  rclcpp::shutdown();return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
