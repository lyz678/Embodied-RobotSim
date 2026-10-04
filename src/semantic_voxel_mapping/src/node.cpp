#include "semantic_voxel_mapping/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <climits>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <limits>
#include <sstream>

using namespace std::chrono_literals;
class SemanticMapNode:public rclcpp::Node {
 using Cloud=sensor_msgs::msg::PointCloud2;using Trigger=std_srvs::srv::Trigger;
 svm::Map map_;std::unordered_set<svm::Color> palette_;std::string signature_,frame_,file_;
 double min_range_,max_range_,zmin_,zmax_,confidence_,integration_rate_,publish_rate_;
 size_t budget_;int device_;int64_t last_stamp_=-1;bool clock_fault_=false,dirty_=false;
 uint64_t integrated_=0,dropped_=0,errors_=0;double gpu_ms_=0,total_ms_=0;size_t scratch_=0;
 std::chrono::steady_clock::time_point last_wall_{};
 std::unique_ptr<tf2_ros::Buffer> tf_;std::unique_ptr<tf2_ros::TransformListener> listener_;
 rclcpp::Subscription<Cloud>::SharedPtr input_;
 rclcpp::Publisher<Cloud>::SharedPtr voxels_;
 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
 rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_;
 rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_;
 rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
 std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
 rclcpp::TimerBase::SharedPtr timer_;
 template<class T>T option(const std::string& name,T value){rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;return declare_parameter(name,value,d);}
 static uint32_t read32(const uint8_t* data,bool swap){uint32_t value;std::memcpy(&value,data,4);return swap?__builtin_bswap32(value):value;}
 static float number(uint32_t bits){float value;std::memcpy(&value,&bits,4);return value;}
public:
 SemanticMapNode():Node("semantic_voxel_map"){
  svm::Config c;c.resolution=option("resolution",.1);c.hit=option("hit",.7);c.miss=option("miss",.4);
  c.clamp_min=option("clamp_min",.12);c.clamp_max=option("clamp_max",.97);c.occupied=option("occupied_threshold",.5);
  int window=option("vote_window",30),minimum=option("min_observations",3),maximum=option("max_voxels",2000000);
  if(window<1||minimum<1||maximum<1)throw std::runtime_error("Invalid integer map limits");
  c.window=window;c.min_observations=minimum;c.majority=option("majority_threshold",.6);c.max_voxels=maximum;c.validate();
  map_=svm::Map(c);frame_=option("map_frame",std::string("odom"));file_=option("map_file",std::string("maps/semantic_map.svm"));
  min_range_=option("min_range",.2);max_range_=option("max_range",8.);zmin_=option("min_z",.1);zmax_=option("max_z",2.5);
  confidence_=option("confidence_threshold",.5);integration_rate_=option("integration_rate",5.);publish_rate_=option("publish_rate",1.);
  int memory=option("gpu_scratch_mib",512);device_=option("cuda_device",0);
  if(!std::isfinite(min_range_)||!std::isfinite(max_range_)||min_range_<0||max_range_<=min_range_||
   !std::isfinite(zmin_)||!std::isfinite(zmax_)||zmax_<=zmin_||!std::isfinite(confidence_)||confidence_<0||confidence_>1||
   !std::isfinite(integration_rate_)||integration_rate_<=0||!std::isfinite(publish_rate_)||publish_rate_<=0||memory<1||memory>4096||c.resolution<.005)
   throw std::runtime_error("Invalid range/rate/height/GPU configuration (resolution must be >= 0.005 m)");
  budget_=size_t(memory)*1024*1024;
  auto palette_file=option("palette_file",std::string(""));
  if(palette_file.empty())palette_file=ament_index_cpp::get_package_share_directory("yoloe_infer")+"/configs/config.yaml";
  auto yaml=YAML::LoadFile(palette_file);std::unordered_set<int> ids;std::vector<std::string> palette_records;
  for(auto cls:yaml["classes"]){auto rgb=cls["color"].as<std::vector<int>>();int id=cls["id"].as<int>();auto name=cls["name"].as<std::string>();
   if(rgb.size()!=3||std::any_of(rgb.begin(),rgb.end(),[](int v){return v<0||v>255;})||!ids.insert(id).second)throw std::runtime_error("Invalid class palette");
   auto color=(uint32_t(rgb[0])<<16)|(uint32_t(rgb[1])<<8)|rgb[2];
   if(color==svm::UNKNOWN||!palette_.insert(color).second)throw std::runtime_error("Class RGB colors must be unique; white is reserved for unknown");
   palette_records.push_back(std::to_string(id)+":"+name+":"+std::to_string(color));
  }
  auto unknown=yaml["default_color"].as<std::vector<int>>();if(unknown!=std::vector<int>({255,255,255})||palette_.empty())throw std::runtime_error("Semantic palette requires white unknown and at least one class");
  std::sort(palette_records.begin(),palette_records.end());std::ostringstream sig;sig<<std::setprecision(17)<<frame_<<' '<<c.resolution<<' '<<c.hit<<' '<<c.miss<<' '<<c.clamp_min<<' '<<c.clamp_max<<' '<<c.occupied<<' '<<c.window<<' '<<c.min_observations<<' '<<c.majority<<' '<<min_range_<<' '<<max_range_<<' '<<zmin_<<' '<<zmax_<<' '<<confidence_;
  for(auto& record:palette_records)sig<<' '<<record;signature_=sig.str();
  svm::require_cuda(device_);
  tf_=std::make_unique<tf2_ros::Buffer>(get_clock());listener_=std::make_unique<tf2_ros::TransformListener>(*tf_);
  auto qos=rclcpp::QoS(1).transient_local().reliable();
  voxels_=create_publisher<Cloud>("/semantic_map/voxels",qos);
  markers_=create_publisher<visualization_msgs::msg::MarkerArray>("/occupied_cells_vis_array",qos);
  octomap_=create_publisher<octomap_msgs::msg::Octomap>("/octomap_full",qos);
  grid_=create_publisher<nav_msgs::msg::OccupancyGrid>("/projected_map",qos);
  diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/semantic_map/diagnostics",10);
  auto topic=option("cloud_topic",std::string("/yoloe_multi_text_prompt/pointcloud_semantic"));
  input_=create_subscription<Cloud>(topic,rclcpp::SensorDataQoS().keep_last(2),[this](Cloud::ConstSharedPtr msg){receive(*msg);});
  service("clear",[this]{map_.cells.clear();clock_fault_=false;last_stamp_=-1;last_wall_={};dirty_=true;});
  service("save",[this]{save();});
  service("load",[this]{std::ifstream in(file_,std::ios::binary);map_.load(in,signature_,palette_);clock_fault_=false;last_stamp_=-1;last_wall_={};dirty_=true;});
  timer_=create_wall_timer(std::chrono::duration<double>(1/publish_rate_),[this]{publish();diagnose();});
  RCLCPP_INFO(get_logger(),"CUDA semantic map: input=%s, resolution=%.3f m, frame=%s, votes=%u, min=%u, majority=%.2f",topic.c_str(),c.resolution,frame_.c_str(),c.window,c.min_observations,c.majority);
 }
private:
 template<class F>void service(const std::string& name,F action){services_.push_back(create_service<Trigger>("~/"+name,[action](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr response){try{action();response->success=true;response->message="OK";}catch(const std::exception& e){response->success=false;response->message=e.what();}}));}
 void save(){
  auto path=std::filesystem::path(file_);if(path.has_parent_path())std::filesystem::create_directories(path.parent_path());
  auto temp=file_+".tmp";try{std::ofstream out(temp,std::ios::binary|std::ios::trunc);map_.save(out,signature_);out.close();if(!out)throw std::runtime_error("Map flush failed");std::filesystem::rename(temp,file_);}catch(...){std::filesystem::remove(temp);throw;}
 }
 void receive(const Cloud& msg){
  auto start=std::chrono::steady_clock::now();int64_t stamp=rclcpp::Time(msg.header.stamp).nanoseconds();
  if(last_stamp_>=0&&stamp<last_stamp_){clock_fault_=true;RCLCPP_ERROR(get_logger(),"Clock rewind: integration paused; clear or load the map after restarting localization");}
  if(clock_fault_||stamp==last_stamp_){++dropped_;return;}
  if(last_wall_.time_since_epoch().count()&&std::chrono::duration<double>(start-last_wall_).count()<1/integration_rate_){++dropped_;return;}
  try{
   if(msg.header.frame_id.empty()||msg.point_step<20||uint64_t(msg.row_step)<uint64_t(msg.width)*msg.point_step||msg.data.size()!=uint64_t(msg.row_step)*msg.height)throw std::runtime_error("Malformed semantic cloud layout");
   int offsets[5]={-1,-1,-1,-1,-1};const char* names[]={"x","y","z","rgb","confidence"};
   for(auto& f:msg.fields)for(int i=0;i<5;++i)if(f.name==names[i]){
    if(f.count!=1||uint64_t(f.offset)+4>msg.point_step||(i==3?(f.datatype!=6&&f.datatype!=7):f.datatype!=7))throw std::runtime_error("Invalid semantic cloud field");offsets[i]=f.offset;
   }
   for(int o:offsets)if(o<0)throw std::runtime_error("Semantic cloud requires x/y/z/rgb/confidence");
   auto transform=tf_->lookupTransform(frame_,msg.header.frame_id,rclcpp::Time(msg.header.stamp),rclcpp::Duration::from_seconds(.5));
   tf2::Transform pose;tf2::fromMsg(transform.transform,pose);auto translation=pose.getOrigin();svm::Vec origin{translation.x(),translation.y(),translation.z()};
   auto check_key=[this](svm::Vec v){for(auto a:{v.x,v.y,v.z})if(!std::isfinite(a)||a/map_.config.resolution<=-svm::BIAS||a/map_.config.resolution>=svm::BIAS)throw std::runtime_error("Point outside voxel key range");};check_key(origin);
   uint16_t endian=1;bool swap=msg.is_bigendian!=(*reinterpret_cast<uint8_t*>(&endian)==0);
   std::vector<svm::Ray> rays;rays.reserve(size_t(msg.width)*msg.height);svm::Frame evidence;
   for(unsigned row=0;row<msg.height;++row)for(unsigned col=0;col<msg.width;++col){
    auto p=msg.data.data()+size_t(row)*msg.row_step+size_t(col)*msg.point_step;
    double x=number(read32(p+offsets[0],swap)),y=number(read32(p+offsets[1],swap)),z=number(read32(p+offsets[2],swap));
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z))continue;
    double distance=std::sqrt(x*x+y*y+z*z);if(distance<min_range_||distance==0)continue;
    auto point=pose*tf2::Vector3(x,y,z);svm::Vec end{point.x(),point.y(),point.z()};
    bool hit=distance<=max_range_&&end.z>=zmin_&&end.z<=zmax_;
    if(distance>max_range_){double ratio=max_range_/distance;end={origin.x+(end.x-origin.x)*ratio,origin.y+(end.y-origin.y)*ratio,origin.z+(end.z-origin.z)*ratio};}
    check_key(end);rays.push_back({origin,end,hit});
    if(hit){auto k=svm::key(end,map_.config.resolution);evidence.hits.insert(k);auto rgb=read32(p+offsets[3],swap)&0xffffff;float score=number(read32(p+offsets[4],swap));
     if(std::isfinite(score)&&score>=confidence_&&score<=1&&palette_.count(rgb))++evidence.votes[k][rgb];}
   }
   auto result=svm::raycast_cuda(rays,map_.config.resolution,zmin_,zmax_,budget_);evidence.free.insert(result.free.begin(),result.free.end());
   map_.commit(evidence);gpu_ms_=result.milliseconds;scratch_=result.peak_scratch_bytes;
   total_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
   last_stamp_=stamp;last_wall_=start;++integrated_;dirty_=true;
  }catch(const std::exception& e){++errors_;++dropped_;RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Whole frame rejected: %s",e.what());}
 }
 void publish(){
  if(!dirty_)return;
  try{
   auto stamp=now();Cloud cloud;cloud.header.frame_id=frame_;cloud.header.stamp=stamp;cloud.height=1;cloud.is_dense=true;
   sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2Fields(6,"x",1,7,"y",1,7,"z",1,7,"rgb",1,6,"semantic_confidence",1,7,"occupancy",1,7);
   size_t occupied=0;for(auto& entry:map_.cells)if(map_.occupied(entry.second))++occupied;modifier.resize(occupied);
   visualization_msgs::msg::Marker marker;marker.header=cloud.header;marker.ns="semantic_voxels";marker.id=0;marker.type=visualization_msgs::msg::Marker::CUBE_LIST;marker.action=visualization_msgs::msg::Marker::ADD;marker.pose.orientation.w=1;marker.scale.x=marker.scale.y=marker.scale.z=map_.config.resolution;
   sensor_msgs::PointCloud2Iterator<float> px(cloud,"x"),py(cloud,"y"),pz(cloud,"z"),prob(cloud,"occupancy"),conf(cloud,"semantic_confidence");sensor_msgs::PointCloud2Iterator<uint32_t> rgb(cloud,"rgb");
   // Octree is a compatibility export only, never the canonical fusion state.
   octomap::ColorOcTree tree(map_.config.resolution);
   std::unordered_map<uint64_t,int> projection;int minx=INT_MAX,miny=INT_MAX,maxx=INT_MIN,maxy=INT_MIN;
   for(auto& entry:map_.cells){auto position=svm::center(entry.first,map_.config.resolution);auto label=map_.label(entry.second);
    auto node=tree.updateNode(octomap::point3d(position.x,position.y,position.z),map_.occupied(entry.second),true);if(!node)throw std::runtime_error("Voxel outside OctoMap export extent");node->setLogOdds(entry.second.odds);node->setColor((label.color>>16)&255,(label.color>>8)&255,label.color&255);
    int ix=svm::coordinate(entry.first,0),iy=svm::coordinate(entry.first,1);minx=std::min(minx,ix);miny=std::min(miny,iy);maxx=std::max(maxx,ix);maxy=std::max(maxy,iy);
    uint64_t xy=(uint64_t(uint32_t(ix))<<32)|uint32_t(iy);auto& value=projection[xy];value=std::max(value,map_.occupied(entry.second)?100:0);
    if(map_.occupied(entry.second)){
     *px=position.x;*py=position.y;*pz=position.z;*rgb=label.color;*conf=label.fraction;*prob=1/(1+std::exp(-entry.second.odds));++px;++py;++pz;++rgb;++conf;++prob;
     geometry_msgs::msg::Point p;p.x=position.x;p.y=position.y;p.z=position.z;marker.points.push_back(p);std_msgs::msg::ColorRGBA color;color.r=((label.color>>16)&255)/255.f;color.g=((label.color>>8)&255)/255.f;color.b=(label.color&255)/255.f;color.a=1;marker.colors.push_back(color);
    }
   }
   static_cast<octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>&>(tree).updateInnerOccupancy();octomap_msgs::msg::Octomap octree;octree.header=cloud.header;octomap_msgs::fullMapToMsg(tree,octree);
   nav_msgs::msg::OccupancyGrid grid;grid.header=cloud.header;grid.info.resolution=map_.config.resolution;grid.info.origin.orientation.w=1;
   if(!projection.empty()){
    auto width=int64_t(maxx)-minx+1,height=int64_t(maxy)-miny+1;if(width*height>16000000)throw std::runtime_error("2D projection exceeds output budget");
    grid.info.width=width;grid.info.height=height;grid.info.origin.position.x=minx*map_.config.resolution;grid.info.origin.position.y=miny*map_.config.resolution;grid.data.assign(width*height,-1);
    for(auto entry:projection){int x=int32_t(entry.first>>32),y=int32_t(entry.first&0xffffffff);grid.data[size_t(y-miny)*width+x-minx]=entry.second;}
   }
   voxels_->publish(cloud);visualization_msgs::msg::MarkerArray array;array.markers.push_back(marker);markers_->publish(array);octomap_->publish(octree);grid_->publish(grid);dirty_=false;
  }catch(const std::exception& e){++errors_;RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"Map export failed: %s",e.what());}
 }
 void diagnose(){
  diagnostic_msgs::msg::DiagnosticArray array;array.header.stamp=now();diagnostic_msgs::msg::DiagnosticStatus status;status.name="semantic_voxel_map";status.hardware_id="cuda:"+std::to_string(device_);status.level=clock_fault_?2:0;status.message=clock_fault_?"clock rewind; reset required":"CUDA active";
  for(auto entry:std::vector<std::pair<std::string,std::string>>{{"integrated_frames",std::to_string(integrated_)},{"dropped_frames",std::to_string(dropped_)},{"errors",std::to_string(errors_)},{"voxels",std::to_string(map_.cells.size())},{"gpu_ms",std::to_string(gpu_ms_)},{"integration_ms",std::to_string(total_ms_)},{"scratch_bytes_upper_bound",std::to_string(scratch_)}}){diagnostic_msgs::msg::KeyValue kv;kv.key=entry.first;kv.value=entry.second;status.values.push_back(kv);}array.status.push_back(status);diagnostics_->publish(array);
 }
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<SemanticMapNode>());}catch(const std::exception& e){std::cerr<<"Semantic CUDA map startup failed: "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
