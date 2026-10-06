// Continuous-controller wrapper for scan-rate localization. The Nav2 shim's
// now-stamped TF lookup can wait for a future LIO sample on every replan.
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "x_bot_control/heading_brake.hpp"

namespace x_bot_control {
// Jazzy Graceful measures the remaining path from its nearest grid point,
// excluding the short robot-to-first-point segment. Ask its private final-turn
// logic to enter XY tolerance slightly later; the server's actual goal checker
// and requested final orientation remain unchanged.
class TrackingGoalChecker : public nav2_core::GoalChecker {
 public:
  TrackingGoalChecker(nav2_core::GoalChecker* checker, double margin)
      : checker_(checker), margin_(margin) {}
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr&,
                  const std::string&,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override { checker_->reset(); }
  bool isGoalReached(const geometry_msgs::msg::Pose& pose,
                     const geometry_msgs::msg::Pose& goal,
                     const geometry_msgs::msg::Twist& velocity) override {
    return checker_->isGoalReached(pose, goal, velocity);
  }
  bool getTolerances(geometry_msgs::msg::Pose& pose,
                     geometry_msgs::msg::Twist& velocity) override {
    if (!checker_->getTolerances(pose, velocity)) return false;
    for (double* value : {&pose.position.x, &pose.position.y}) {
      if (*value > 0.) *value = std::max(*value * .5, *value - margin_);
    }
    return true;
  }
 private:
  nav2_core::GoalChecker* checker_;
  double margin_;
};

class ForwardHeadingController
    : public nav2_rotation_shim_controller::RotationShimController {
 public:
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    RotationShimController::configure(parent, name, tf, costmap);
    auto node = parent.lock();
    const auto alignment_key = name + ".enable_heading_alignment";
    if (!node->has_parameter(alignment_key))
      node->declare_parameter(alignment_key, false);
    align_heading_ = node->get_parameter(alignment_key).as_bool();
    const auto key = name + ".heading_tf_max_age";
    if (!node->has_parameter(key)) node->declare_parameter(key, 0.5);
    max_age_ = node->get_parameter(key).as_double();
    auto parameter = [&](const std::string& suffix, double value) {
      const auto parameter_name = name + "." + suffix;
      if (!node->has_parameter(parameter_name))
        node->declare_parameter(parameter_name, value);
      return node->get_parameter(parameter_name).as_double();
    };
    brake_accel_ = parameter("heading_braking_accel", 2.0);
    heading_gain_ = parameter("heading_gain", 2.0);
    brake_latency_ = parameter("heading_braking_latency", 0.2);
    settle_angle_ = parameter("heading_settle_angle", 0.12);
    release_rate_ = parameter("heading_release_rate", 0.2);
    max_forward_ = parameter("vx_max", 1.0);
    max_yaw_ = parameter("wz_max", 1.2);
    goal_margin_ = parameter("tracking_goal_tolerance_margin", 0.0);
    if (!std::isfinite(goal_margin_) || goal_margin_ < 0.)
      throw std::invalid_argument("Invalid tracking goal tolerance margin");
    for (double value : {brake_accel_, heading_gain_, brake_latency_,
                         settle_angle_, release_rate_, max_forward_, max_yaw_}) {
      if (!std::isfinite(value) || value <= 0.)
        throw std::invalid_argument("Invalid heading braking parameter");
    }
    if (!std::isfinite(max_age_) || max_age_ <= 0.)
      throw std::invalid_argument("heading_tf_max_age must be positive");
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose,
      const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker* goal_checker) override {
    // Use the existing shim's collision-checked rotational command and ramp.
    // Continuous heading checks also cover moving beyond an old plan's start.
    {
      auto* costmap = costmap_ros_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(
          *costmap->getMutex());
      std::lock_guard<std::mutex> plan_lock(mutex_);
      if (current_path_.poses.empty())
        throw nav2_core::InvalidPath("Empty path");
      auto robot = latestPose(pose, current_path_.header.frame_id);
      geometry_msgs::msg::Pose position_tolerance;
      geometry_msgs::msg::Twist velocity_tolerance;
      const auto& goal_position = current_path_.poses.back().pose.position;
      const double goal_distance =
          std::hypot(goal_position.x - robot.pose.position.x,
                     goal_position.y - robot.pose.position.y);
      const bool near_goal =
          goal_checker &&
          goal_checker->getTolerances(position_tolerance, velocity_tolerance) &&
          goal_distance <= position_tolerance.position.x;
      // Inside XY tolerance, the primary finishes the requested goal orientation;
      // a point a few centimetres behind the body is not a new path heading.
      if (!near_goal) {
        size_t nearest = 0;
        double best = std::numeric_limits<double>::max();
        for (size_t i = 0; i < current_path_.poses.size(); ++i) {
          const auto& p = current_path_.poses[i].pose.position;
          const double distance = std::hypot(p.x - robot.pose.position.x,
                                             p.y - robot.pose.position.y);
          if (distance < best) {
            best = distance;
            nearest = i;
          }
        }
        size_t target = nearest;
        double arc = 0.;
        while (target + 1 < current_path_.poses.size() &&
               arc < forward_sampling_distance_) {
          const auto& a = current_path_.poses[target].pose.position;
          const auto& b = current_path_.poses[++target].pose.position;
          arc += std::hypot(b.x - a.x, b.y - a.y);
        }
        auto point = current_path_.poses[target];
        point.header.frame_id = current_path_.header.frame_id;
        const auto base = latestPose(point, costmap_ros_->getBaseFrameID());
        const double error =
            std::atan2(base.pose.position.y, base.pose.position.x);
        const double threshold = in_rotation_ ? angular_disengage_threshold_
                                              : angular_dist_threshold_;
        // Do not hand off to fast forward motion while the body is still
        // spinning.
        const bool settling =
            in_rotation_ && std::abs(velocity.angular.z) > release_rate_;
        if (align_heading_ && (std::abs(error) > threshold || settling)) {
          // A blocked rotation must not override primary-controller collision
          // avoidance.
          try {
            auto cmd = computeRotateToHeadingCommand(error, pose, velocity);
            const double target = headingBrakeRate(
                error, velocity.angular.z, rotate_to_heading_angular_vel_,
                brake_accel_, heading_gain_, brake_latency_, settle_angle_);
            const double braking_step = brake_accel_ * control_duration_;
            const double previous =
                last_angular_vel_ == std::numeric_limits<double>::max()
                    ? 0.
                    : last_angular_vel_;
            const double ramped = std::clamp(target, previous - braking_step,
                                             previous + braking_step);
            // Only reduce the collision-checked command, never change its sign.
            const double magnitude = std::min(
                std::abs(cmd.twist.angular.z),
                ramped * cmd.twist.angular.z > 0. ? std::abs(ramped) : 0.);
            cmd.twist.angular.z = std::copysign(magnitude, cmd.twist.angular.z);
            in_rotation_ = true;
            last_angular_vel_ = cmd.twist.angular.z;
            return cmd;
          } catch (const nav2_core::NoValidControl&) {
            // The primary may find a collision-free moving turn instead.
          }
        }
      }
      in_rotation_ = false;
      path_updated_ = false;
    }
    TrackingGoalChecker tracking_checker(goal_checker, goal_margin_);
    auto cmd = primary_controller_->computeVelocityCommands(
        pose, velocity, goal_checker && goal_margin_ > 0.
                            ? &tracking_checker : goal_checker);
    // MPPI's polynomial output smoothing can overshoot either velocity bound.
    // Clamp tiny reverse undershoot, then scale uniformly to preserve curvature.
    cmd.twist.linear.x = std::max(0., cmd.twist.linear.x);
    const double scale = std::max(
        {1., cmd.twist.linear.x / max_forward_,
         std::abs(cmd.twist.angular.z) / max_yaw_});
    cmd.twist.linear.x /= scale;
    cmd.twist.angular.z /= scale;
    last_angular_vel_ = cmd.twist.angular.z;
    return cmd;
  }

 private:
  geometry_msgs::msg::PoseStamped latestPose(
      geometry_msgs::msg::PoseStamped input, const std::string& frame) {
    input.header.stamp = builtin_interfaces::msg::Time();
    try {
      // A latest lookup never waits for a nonexistent future scan. Bound age
      // explicitly; stale localization must still fail closed.
      auto transform = tf_->lookupTransform(frame, input.header.frame_id,
                                            tf2::TimePointZero);
      if (frame != input.header.frame_id &&
          (transform.header.stamp.sec != 0 ||
           transform.header.stamp.nanosec != 0)) {
        const double age =
            (clock_->now() -
             rclcpp::Time(transform.header.stamp, clock_->get_clock_type()))
                .seconds();
        if (age < -0.05 || age > max_age_)
          throw nav2_core::ControllerTFError(
              "Heading transform is stale or in the future");
      }
      geometry_msgs::msg::PoseStamped output;
      tf2::doTransform(input, output, transform);
      return output;
    } catch (const tf2::TransformException& ex) {
      throw nav2_core::ControllerTFError(ex.what());
    }
  }
  double max_age_{0.5}, brake_accel_{2.}, heading_gain_{2.},
      brake_latency_{0.2}, settle_angle_{0.12}, release_rate_{0.2};
  double max_forward_{1.}, max_yaw_{1.2};
  bool align_heading_{false};
  double goal_margin_{0.};
};
}  // namespace x_bot_control
PLUGINLIB_EXPORT_CLASS(x_bot_control::ForwardHeadingController,
                       nav2_core::Controller)
