// Remove robot collision geometry before VIO depth enters mapping/navigation.
#include <x_bot_common/self_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <limits>

class VioCloudFilter : public rclcpp::Node {
 public:
  VioCloudFilter() : Node("vio_cloud_filter"), tf_(get_clock()), listener_(tf_) {
    padding_ = declare_parameter<double>("self_filter_padding", .02);
    description_ = create_subscription<std_msgs::msg::String>(
        "/robot_description", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          try { filter_.load(msg->data, padding_); }
          catch (const std::exception &e) { RCLCPP_ERROR(get_logger(), "%s", e.what()); }
        });
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/vio/cloud_odom", 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/x_bot/scan", 10);
    cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/vio/cloud_unfiltered", 10,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { receive(*msg); });
  }
 private:
  void receive(const sensor_msgs::msg::PointCloud2 &input) {
    try {
      const auto mask = filter_.mask(input, tf_);
      const auto t = tf_.lookupTransform("base_footprint", input.header.frame_id,
          rclcpp::Time(input.header.stamp), rclcpp::Duration::from_seconds(.1)).transform;
      const Eigen::Quaterniond rotation(t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z);
      const Eigen::Vector3d translation(t.translation.x,t.translation.y,t.translation.z);
      sensor_msgs::msg::PointCloud2 output;
      output.header = input.header;
      sensor_msgs::PointCloud2Modifier modifier(output);
      modifier.setPointCloud2FieldsByString(1,"xyz");
      std::vector<Eigen::Vector3f> points;
      points.reserve(mask.size());
      sensor_msgs::msg::LaserScan scan;
      scan.header = input.header;
      scan.header.frame_id = "base_footprint";
      constexpr double pi = 3.14159265358979323846;
      scan.angle_min = -pi;
      scan.angle_increment = 2*pi/720;
      scan.angle_max = scan.angle_min+719*scan.angle_increment;
      scan.range_min = .1;
      scan.range_max = 10.;
      scan.scan_time = .1;
      scan.ranges.assign(720,std::numeric_limits<float>::infinity());
      sensor_msgs::PointCloud2ConstIterator<float> x(input,"x"),y(input,"y"),z(input,"z");
      for (size_t i=0;i<mask.size();++i,++x,++y,++z) {
        if (mask[i] != point_containment_filter::ShapeMask::OUTSIDE) continue;
        Eigen::Vector3f p(*x,*y,*z);
        if (!p.allFinite()) continue;
        points.push_back(p);
        const Eigen::Vector3d local = rotation*p.cast<double>()+translation;
        const double r=std::hypot(local.x(),local.y());
        if (local.z()<.1 || local.z()>1.8 || r<.1 || r>10.) continue;
        const size_t index=std::min(size_t(719),size_t((std::atan2(local.y(),local.x())+pi)/scan.angle_increment));
        scan.ranges[index]=std::min(scan.ranges[index],float(r));
      }
      if (points.empty()) return;
      modifier.resize(points.size());
      sensor_msgs::PointCloud2Iterator<float> ox(output,"x"),oy(output,"y"),oz(output,"z");
      for (const auto &p:points) { *ox=p.x();*oy=p.y();*oz=p.z();++ox;++oy;++oz; }
      cloud_pub_->publish(output);
      scan_pub_->publish(scan);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Waiting for self-filter geometry/TF: %s",e.what());
    }
  }
  double padding_;
  RobotSelfFilter filter_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<VioCloudFilter>());
  rclcpp::shutdown();
}
