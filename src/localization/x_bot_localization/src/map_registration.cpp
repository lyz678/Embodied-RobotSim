#include "registration.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <map>
#include <sstream>

using Matrix = Eigen::Matrix4f;
Matrix pose_matrix(const geometry_msgs::msg::Pose &p) {
  Matrix m = Matrix::Identity();
  Eigen::Quaternionf q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1e-6) throw std::runtime_error("Invalid quaternion");
  m.block<3,3>(0,0) = q.normalized().toRotationMatrix();
  m.block<3,1>(0,3) << p.position.x,p.position.y,p.position.z;
  if (!m.allFinite()) throw std::runtime_error("Nonfinite pose");
  return m;
}

class Registration : public rclcpp::Node {
public:
  Registration() : Node("map_registration"), map_(new xbot::Cloud) {
    const auto path = declare_parameter<std::string>("pcd_map", "");
    if (path.empty() || pcl::io::loadPCDFile(path, *map_) < 0 || map_->size() < 100)
      throw std::runtime_error("Missing/invalid paired PCD map: " + path);
    std::vector<int> indices;
    map_->is_dense = false;
    pcl::removeNaNFromPointCloud(*map_, *map_, indices);
    map_ = xbot::voxel(map_, .1f);
    radius_ = declare_parameter<double>("crop_radius", 45.);
    max_rmse_ = declare_parameter<double>("max_rmse", .15);
    min_overlap_ = declare_parameter<double>("min_overlap", .65);
    const auto initial = declare_parameter<std::vector<double>>("initial_pose", {0.,0.,0.});
    if (initial.size() != 3) throw std::runtime_error("initial_pose must be [x,y,yaw]");
    seed_.block<3,3>(0,0) = Eigen::AngleAxisf(initial[2], Eigen::Vector3f::UnitZ()).toRotationMatrix();
    seed_(0,3) = initial[0]; seed_(1,3) = initial[1];
    pending_seed_ = declare_parameter<bool>("use_initial_pose", true);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("/localization/valid", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/localization/status", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 30,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto ns = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (latest_ns_ && ns <= latest_ns_) { fault_ = true; report(false,"clock_rewind_restart_required"); return; }
        latest_ns_ = ns;
        latest_odom_ = pose_matrix(msg->pose.pose);
        poses_[ns] = latest_odom_;
        while (poses_.size()>50) poses_.erase(poses_.begin());
        if (pending_seed_) { correction_ = seed_ * latest_odom_.inverse(); pending_seed_=false; seeded_=true; }
        auto pending = pending_clouds_.find(ns);
        if (pending != pending_clouds_.end()) {
          auto cloud = pending->second;
          pending_clouds_.erase(pending);
          scan(cloud);
        }
      });
    init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        if (msg->header.frame_id != "map") { report(false,"initialpose_must_be_in_map"); return; }
        try { seed_=pose_matrix(msg->pose.pose); }
        catch (const std::exception &e) { report(false,e.what()); return; }
        // RViz pose describes BASE at receipt, not IMU and not odom origin.
        if (latest_ns_) { correction_=seed_*latest_odom_.inverse(); seeded_=true; }
        else pending_seed_=true;
        successes_=0; failures_=0;
        report(false,"initializing");
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("/fastlio/cloud_odom", 10,
      std::bind(&Registration::scan, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      if (std::chrono::steady_clock::now()-last_scan_ > std::chrono::seconds(2)) report(false,"stale_scan");
    });
  }
private:
  void report(bool valid, const std::string &text) {
    std_msgs::msg::Bool msg; msg.data=valid; valid_pub_->publish(msg);
    std_msgs::msg::String status; status.data=text; status_pub_->publish(status);
  }
  void scan(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    last_scan_ = std::chrono::steady_clock::now();
    if (fault_ || !seeded_) { report(false,"waiting_for_initialpose_or_restart"); return; }
    if (msg->header.frame_id != "odom") { report(false,"bad_cloud_frame"); return; }
    auto it = poses_.find(rclcpp::Time(msg->header.stamp).nanoseconds());
    if (it == poses_.end()) {
      // DDS ordering across two topics is not guaranteed, even from one node.
      pending_clouds_[rclcpp::Time(msg->header.stamp).nanoseconds()] = msg;
      while (pending_clouds_.size()>30) pending_clouds_.erase(pending_clouds_.begin());
      return;
    }
    if ((now()-rclcpp::Time(msg->header.stamp)).seconds() > .5) {
      report(false,"stale_cloud"); return;
    }
    auto scan = std::make_shared<xbot::Cloud>();
    pcl::fromROSMsg(*msg, *scan);
    std::vector<int> indices;
    scan->is_dense=false;
    pcl::removeNaNFromPointCloud(*scan,*scan,indices);
    const Matrix base = correction_ * it->second;
    auto local_map = std::make_shared<xbot::Cloud>();
    const Eigen::Vector3f center=base.block<3,1>(0,3);
    for (const auto &p : *map_) if ((p.getVector3fMap()-center).norm() < radius_) local_map->push_back(p);
    auto result = xbot::align(scan, local_map, correction_, max_rmse_, min_overlap_);
    std::ostringstream detail;
    detail << (result.valid?"tracking":"rejected") << " rmse=" << result.rmse << " overlap=" << result.overlap;
    if (!result.valid) {
      successes_=0;
      if (++failures_>=5) seeded_=false; // require deliberate reseed, not blind global search
      report(false,detail.str()); return;
    }
    failures_=0;
    correction_=result.pose;
    if (++successes_<3) { report(false,"confirming_"+detail.str()); return; }
    geometry_msgs::msg::TransformStamped tf;
    tf.header=msg->header; tf.header.frame_id="map"; tf.child_frame_id="odom";
    tf.transform.translation.x=correction_(0,3);
    tf.transform.translation.y=correction_(1,3);
    tf.transform.translation.z=correction_(2,3);
    Eigen::Quaternionf q(correction_.block<3,3>(0,0)); q.normalize();
    tf.transform.rotation.x=q.x(); tf.transform.rotation.y=q.y();
    tf.transform.rotation.z=q.z(); tf.transform.rotation.w=q.w();
    tf_->sendTransform(tf);
    report(true,detail.str());
  }
  xbot::Cloud::Ptr map_;
  Matrix correction_=Matrix::Identity(), seed_=Matrix::Identity(), latest_odom_=Matrix::Identity();
  std::map<int64_t,Matrix> poses_;
  std::map<int64_t,sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_clouds_;
  int64_t latest_ns_=0;
  bool pending_seed_=false, seeded_=false, fault_=false;
  int successes_=0,failures_=0;
  double radius_,max_rmse_,min_overlap_;
  std::chrono::steady_clock::time_point last_scan_=std::chrono::steady_clock::now();
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<Registration>()); }
  catch (const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("map_registration"),"%s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
