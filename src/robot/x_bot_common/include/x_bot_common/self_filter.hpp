#pragma once
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/point_containment_filter/shape_mask.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <tf2_ros/buffer.h>
#include <unordered_map>
#include <limits>

// Filter the robot's collision geometry before raycasting, keeping semantic
// mapping and the exported MoveIt scene free of the robot's own measurements.
class RobotSelfFilter {
 using Handle = point_containment_filter::ShapeHandle;
 struct Shape {std::string link; Eigen::Isometry3d origin;};
 std::unordered_map<Handle, Shape> shapes_;
 std::unordered_map<Handle, Eigen::Isometry3d> transforms_;
 point_containment_filter::ShapeMask mask_;
 moveit::core::RobotModelPtr model_;
public:
 RobotSelfFilter():mask_([this](Handle h, Eigen::Isometry3d& pose){
   auto it=transforms_.find(h);if(it==transforms_.end())return false;pose=it->second;return true;}){}
 bool ready() const {return bool(model_);}
 void load(const std::string& xml,double padding){
   if(ready())return;
   auto urdf=urdf::parseURDF(xml);if(!urdf)throw std::runtime_error("Invalid robot description for self filter");
   auto srdf=std::make_shared<srdf::Model>();srdf->initString(*urdf,"<robot name=\""+urdf->getName()+"\"/>");
   model_=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
   for(auto link:model_->getLinkModels())for(size_t i=0;i<link->getShapes().size();++i){
     auto h=mask_.addShape(link->getShapes()[i],1.0,padding);
     shapes_[h]={link->getName(),link->getCollisionOriginTransforms()[i]};
   }
 }
 std::vector<int> mask(const sensor_msgs::msg::PointCloud2& cloud,tf2_ros::Buffer& tf){
   if(!ready())throw std::runtime_error("Waiting for robot_description before self-filtered mapping");
   std::unordered_map<std::string,Eigen::Isometry3d> links;transforms_.clear();
   for(auto& entry:shapes_){auto& shape=entry.second;
     if(!links.count(shape.link)){
       auto t=tf.lookupTransform(cloud.header.frame_id,shape.link,rclcpp::Time(cloud.header.stamp),rclcpp::Duration::from_seconds(.1));
       auto& q=t.transform.rotation;auto& p=t.transform.translation;
       Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
       pose.linear()=Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
       pose.translation()=Eigen::Vector3d(p.x,p.y,p.z);links[shape.link]=pose;
     }
     transforms_[entry.first]=links.at(shape.link)*shape.origin;
   }
   std::vector<int> result;
   mask_.maskContainment(cloud,Eigen::Vector3d::Zero(),0.0,std::numeric_limits<double>::max(),result);
   return result;
 }
};
