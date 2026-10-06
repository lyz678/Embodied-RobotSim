#include <memory>
#include <thread>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <iostream>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class RobotActionsNode : public rclcpp::Node
{
public:
  RobotActionsNode() : Node("robot_actions")
  {
    // Use ReentrantCallbackGroup to allow concurrent callbacks (crucial for calling actions inside services)
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Define services with callback group
    go_home_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/robot_actions/go_home", 
      std::bind(&RobotActionsNode::go_home_callback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    scan_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/robot_actions/scan", 
      std::bind(&RobotActionsNode::scan_callback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    // Support for direct Pose control (merged from move_to_point)
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/arm_command/pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { topic_callback(msg, false); }, sub_opt);

    cartesian_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/arm_command/cartesian_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { topic_callback(msg, true); }, sub_opt);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&RobotActionsNode::joint_callback, this, std::placeholders::_1), sub_opt);

    status_pub_ = this->create_publisher<std_msgs::msg::String>("/arm_command/status", 10);
    
    
    // Initialize Trajectory Action Client with callback group
    trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this, "/fr3_arm_controller/follow_joint_trajectory", callback_group_);

    // Initialize Trajectory Action Client with callback group
    trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this, "/fr3_arm_controller/follow_joint_trajectory", callback_group_);



    RCLCPP_INFO(this->get_logger(), "Robot Actions Node Initialized (Services + Pose Control + Direct Action)");
  }

  void init_move_group()
  {
    if (move_group_) return;
    
    // Run in a separate thread to avoid blocking construction if MoveGroup needs time
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "fr3_arm");
    
    move_group_->setPlanningPipelineId("ompl");
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(10.0);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.25);
    move_group_->setPoseReferenceFrame(move_group_->getPlanningFrame());
    
    // Explicitly set the TCP to our new center point
    // This ensures we reach for the ball with the fingers, not the wrist
    move_group_->setEndEffectorLink("fingers_center");
    
    RCLCPP_INFO(this->get_logger(), "MoveGroup Interface READY");
    RCLCPP_INFO(this->get_logger(), "End Effector Link: %s", move_group_->getEndEffectorLink().c_str());
    
    // Initialize a local monitor for start-state and collision diagnostics
    RCLCPP_INFO(this->get_logger(), "Initializing PlanningSceneMonitor...");
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), "robot_description");
    
    if (planning_scene_monitor_) {
      planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
      planning_scene_monitor_->startWorldGeometryMonitor();
      planning_scene_monitor_->startStateMonitor();
      
      // The semantic planning export excludes only the selected target.
      // Preserve finger collisions with all other measured obstacles.
      RCLCPP_INFO(this->get_logger(), "PlanningSceneMonitor configured successfully");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to create PlanningSceneMonitor");
    }


  }



private:
  // --- Service Callbacks ---
  void go_home_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!ensure_move_group()) {
        response->success = false;
        response->message = "MoveGroup not ready";
        return;
    }
    
    if (perform_go_home()) {
        response->success = true;
        response->message = "Go Home Executed Successfully";
    } else {
        response->success = false;
        response->message = "Go Home Failed";
    }
  }

  void scan_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!ensure_move_group()) {
        response->success = false;
        response->message = "MoveGroup not ready";
        return;
    }

    if (perform_scan()) {
        response->success = true;
        response->message = "Scan Sequence Executed Successfully";
    } else {
        response->success = false;
        response->message = "Scan Sequence Failed";
    }
  }

  bool plan_with_retries(moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    // Time parameterization can expose collisions between sparse OMPL samples.
    // Retry another fully validated path; never execute an invalid response.
    for (int attempt = 0; attempt < 3; ++attempt) {
      auto result = move_group_->plan(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS) return true;
      RCLCPP_WARN(get_logger(), "Pose planning attempt %d failed: %d", attempt+1, result.val);
    }
    return false;
  }

  // --- Topic Callbacks ---
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    latest_joint_state_ = *msg;
  }
  
  void print_current_pose()
  {
    try {
        auto transform = tf_buffer_->lookupTransform(
            "odom", "fingers_center", 
            tf2::TimePointZero);
        
        auto& t = transform.transform.translation;
        auto& r = transform.transform.rotation;
        
        RCLCPP_INFO(this->get_logger(), 
            "End Effector Pose (odom): pos(%.3f, %.3f, %.3f) ori(%.3f, %.3f, %.3f, %.3f)",
            t.x, t.y, t.z, r.x, r.y, r.z, r.w);
            
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

  void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, bool cartesian)
  {
    std::lock_guard<std::mutex> motion_lock(motion_mutex_);
    if (!ensure_move_group()) {
        RCLCPP_WARN(this->get_logger(), "MoveGroup not yet initialized or failed!");
        return;
    }

    geometry_msgs::msg::PoseStamped target_pose_planning;
    try {
        if (!tf_buffer_->canTransform(move_group_->getPlanningFrame(), msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(1.0))) {
             RCLCPP_WARN(this->get_logger(), "Wait for transform failed");
             return;
        }
        target_pose_planning = tf_buffer_->transform(*msg, move_group_->getPlanningFrame());
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform to planning frame failed: %s", ex.what());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received target pose (planning frame): Pos(%.3f, %.3f, %.3f) Ori(%.3f, %.3f, %.3f, %.3f)",
        target_pose_planning.pose.position.x, target_pose_planning.pose.position.y, target_pose_planning.pose.position.z,
        target_pose_planning.pose.orientation.x, target_pose_planning.pose.orientation.y,
        target_pose_planning.pose.orientation.z, target_pose_planning.pose.orientation.w);
    
    RCLCPP_INFO(this->get_logger(), "=== Before Motion ===");
    print_current_pose();

    if (!prepare_planning_start()) {
        std_msgs::msg::String status;
        status.data = "FAILED: Invalid or unavailable joint state";
        status_pub_->publish(status);
        return;
    }
    // Pass PoseStamped directly so MoveIt handles frame transform
    move_group_->setPoseTarget(target_pose_planning);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    if (cartesian) {
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double fraction = move_group_->computeCartesianPath({target_pose_planning.pose}, 0.005, trajectory, true);
      auto state = move_group_->getCurrentState();
      if (fraction >= 0.99 && state) {
        robot_trajectory::RobotTrajectory timed(move_group_->getRobotModel(), "fr3_arm");
        timed.setRobotTrajectoryMsg(*state, trajectory);
        trajectory_processing::TimeOptimalTrajectoryGeneration timing;
        std::unordered_map<std::string,double> velocities, accelerations;
        auto limits = YAML::LoadFile(ament_index_cpp::get_package_share_directory("franka_fr3_moveit_config") + "/config/fr3_joint_limits.yaml")["joint_limits"];
        for (const auto& name : state->getJointModelGroup("fr3_arm")->getVariableNames()) {
          velocities[name] = move_group_->getRobotModel()->getVariableBounds(name).max_velocity_;
          accelerations[name] = limits[name]["max_acceleration"].as<double>();
        }
        success = timing.computeTimeStamps(timed, velocities, accelerations, 0.3, 0.25);
        timed.getRobotTrajectoryMsg(plan.trajectory);
      }
      RCLCPP_INFO(get_logger(), "Collision-checked Cartesian path fraction: %.3f", fraction);
      if (!success) {
        moveit_msgs::msg::RobotTrajectory diagnostic;
        const double unchecked = move_group_->computeCartesianPath({target_pose_planning.pose}, 0.005, diagnostic, false);
        RCLCPP_WARN(get_logger(), "Straight path fraction without collision checking (diagnostic only): %.3f", unchecked);
        if (state && planning_scene_monitor_) {
          planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
          auto candidate = *state;
          for (const auto& point : diagnostic.joint_trajectory.points) {
            candidate.setVariablePositions(diagnostic.joint_trajectory.joint_names, point.positions);
            candidate.update();
            collision_detection::CollisionRequest request;
            request.group_name = "fr3_arm";request.contacts = true;request.max_contacts = 20;
            collision_detection::CollisionResult result;
            scene->checkCollision(request, result, candidate);
            if (result.collision) {
              for (const auto& pair : result.contacts)
                RCLCPP_WARN(get_logger(), "Straight path blocked by %s / %s", pair.first.first.c_str(), pair.first.second.c_str());
              break;
            }
          }
        }
        RCLCPP_WARN(get_logger(), "Straight path unavailable; trying collision-checked joint-space approach");
        success = plan_with_retries(plan);
      }
    } else {
      success = plan_with_retries(plan);
    }

    if (success) {
        RCLCPP_INFO(this->get_logger(), "Plan valid. Executing...");
        auto exec_result = move_group_->execute(plan);
        
        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Execution DONE.");
            
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "SUCCESS";
            status_pub_->publish(status_msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
            RCLCPP_INFO(this->get_logger(), "=== After Motion ===");
            print_current_pose();
        } else {
             RCLCPP_ERROR(this->get_logger(), "Execution FAILED");
             auto status_msg = std_msgs::msg::String();
             status_msg.data = "FAILED: Execution Error";
             status_pub_->publish(status_msg);
        }
            
    } else {
        RCLCPP_ERROR(this->get_logger(), "Planning FAILED");
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "FAILED: Planning Error";
        status_pub_->publish(status_msg);
    }
  }

  // --- Logic Implementation ---

  bool ensure_move_group() {
      if (!move_group_) {
          init_move_group();
      }
      return (move_group_ != nullptr);
  }

  bool prepare_planning_start() {
      auto state = move_group_->getCurrentState(2.0);
      if (!state) {
          RCLCPP_ERROR(this->get_logger(), "No current joint state for planning");
          return false;
      }
      // PhysX publishes float joint positions. At a hard stop, rounding can
      // exceed the double URDF bound by ~1e-7 rad and fail Jazzy's strict check.
      // Correct only numerical overshoot in the planning copy, never hardware
      // feedback or meaningful out-of-bounds states. Collision checks stay on.
      const auto model = state->getRobotModel();
      for (const auto& name : model->getVariableNames()) {
          const auto& bounds = model->getVariableBounds(name);
          const double value = state->getVariablePosition(name);
          if (!std::isfinite(value)) return false;
          if (!bounds.position_bounded_) continue;
          const double bounded = std::clamp(value, bounds.min_position_, bounds.max_position_);
          const double error = std::abs(value - bounded);
          if (error > 1e-4) {
              RCLCPP_ERROR(this->get_logger(), "%s outside joint limits by %.9f; refusing plan",
                           name.c_str(), error);
              return false;
          }
          if (error > 0.0) {
              RCLCPP_INFO(this->get_logger(), "Correcting numerical start-state overshoot: %s %.9f rad",
                          name.c_str(), error);
              state->setVariablePosition(name, bounded);
          }
      }
      state->update();
      moveit_msgs::msg::RobotState start;
      moveit::core::robotStateToRobotStateMsg(*state, start);
      // Preserve payloads attached in the server's planning scene. A full
      // joint-only state would silently clear them before collision checking.
      start.is_diff = true;
      start.attached_collision_objects.clear(); // Server owns attachment lifecycle; never replay a stale local copy.
      move_group_->setStartState(start);
      return true;
  }

  bool perform_go_home() {
      std::lock_guard<std::mutex> motion_lock(motion_mutex_);
      RCLCPP_INFO(this->get_logger(), "Executing Go Home...");
      if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(30))) {
          RCLCPP_ERROR(this->get_logger(), "Arm trajectory controller not ready for go home");
          return false;
      }
      
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("x_bot_control");
      std::string yaml_file = package_share_directory + "/config/initial_positions.yaml";
      
      std::map<std::string, double> home_joints;
      std::ifstream file(yaml_file);
      
      if (!file.is_open()) {
          RCLCPP_ERROR(this->get_logger(), "Could not open config file: %s", yaml_file.c_str());
          return false;
      }

      std::string line;
      while (std::getline(file, line)) {
          size_t colon_pos = line.find(':');
          if (colon_pos != std::string::npos) {
              std::string key = line.substr(0, colon_pos);
              std::string value_str = line.substr(colon_pos + 1);
              
              size_t first = key.find_first_not_of(" \t");
              if (first == std::string::npos) continue;
              size_t last = key.find_last_not_of(" \t");
              key = key.substr(first, (last - first + 1));
              
              if (key.find("fr3_joint") != std::string::npos) {
                  try {
                      double val = std::stod(value_str);
                      home_joints[key] = val;
                  } catch (...) {}
              }
          }
      }
      file.close();

      if (home_joints.empty()) {
          RCLCPP_ERROR(this->get_logger(), "No joints found in %s", yaml_file.c_str());
          return false;
      }

      move_group_->clearPoseTargets();
      if (!prepare_planning_start()) return false;
      move_group_->setJointValueTarget(home_joints);
      move_group_->setGoalTolerance(0.01);
      
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto planning_result = move_group_->plan(plan);
      if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
          const auto execution_result = move_group_->execute(plan);
          if (execution_result != moveit::core::MoveItErrorCode::SUCCESS)
              RCLCPP_ERROR(this->get_logger(), "Home execution failed: MoveIt code %d", execution_result.val);
          return execution_result == moveit::core::MoveItErrorCode::SUCCESS;
      } else {
          RCLCPP_ERROR(this->get_logger(), "Planning to home failed: MoveIt code %d", planning_result.val);
          return false;
      }
  }



// Class definition update
// ... (Adding client member)
// private:
//   rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_client_;

  std::mutex scan_mutex_;

  bool perform_scan() {
      // Prevent concurrent scans
      std::unique_lock<std::mutex> lock(scan_mutex_, std::defer_lock);
      if (!lock.try_lock()) {
          RCLCPP_WARN(this->get_logger(), "Scan request rejected: Scan already in progress.");
          return false;
      }
      std::lock_guard<std::mutex> motion_lock(motion_mutex_);

      RCLCPP_INFO(this->get_logger(), "Executing Scan Sequence (Sequential Control)...");

      if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(2))) {
          RCLCPP_ERROR(this->get_logger(), "Trajectory action server not available");
          return false;
      }

      std::vector<double> targets = {-1.57, 1.57, 0.0};
      
      for (size_t i = 0; i < targets.size(); ++i) {
          double target_val = targets[i];
          RCLCPP_INFO(this->get_logger(), "--- Scan Step %zu / %zu : Target J1 = %.2f ---", i+1, targets.size(), target_val);

          // ... (Joint State Logic Unchanged) ...
          // --- 1. Get Current Joints ---
          std::map<std::string, double> current_positions;
          {
              std::lock_guard<std::mutex> param_lock(joint_mutex_);
              if (latest_joint_state_.name.empty()) {
                  RCLCPP_ERROR(this->get_logger(), "No joint state received yet");
                  return false;
              }
              for (size_t k = 0; k < latest_joint_state_.name.size(); ++k) {
                  current_positions[latest_joint_state_.name[k]] = latest_joint_state_.position[k];
              }
          }

          auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
          goal_msg.trajectory.joint_names = {
              "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
              "fr3_joint5", "fr3_joint6", "fr3_joint7"
          };

          std::vector<double> target_positions;
          for (const auto& name : goal_msg.trajectory.joint_names) {
              if (name == "fr3_joint1") {
                  target_positions.push_back(target_val);
              } else {
                  if (current_positions.count(name)) 
                      target_positions.push_back(current_positions.at(name));
                  else 
                      target_positions.push_back(0.0);
              }
          }

          trajectory_msgs::msg::JointTrajectoryPoint point;
          point.positions = target_positions;
          
          const double speed = 0.8; // Peak rad/s for a smooth rest-to-rest scan.
          double max_diff = 0.0;
          for (size_t k = 0; k < target_positions.size(); ++k) {
              double current_val = 0.0;
              std::string joint_name = goal_msg.trajectory.joint_names[k];
              if (current_positions.count(joint_name)) {
                  current_val = current_positions.at(joint_name);
              }
              double diff = std::abs(target_positions[k] - current_val);
              if (diff > max_diff) max_diff = diff;
          }
          
          // Quintic interpolation peaks at 1.875 * displacement / duration.
          // Explicit rest endpoints avoid the old abrupt constant-speed start.
          double duration = std::max(1.875 * max_diff / speed, 1.5);
          trajectory_msgs::msg::JointTrajectoryPoint start;
          for (const auto& name : goal_msg.trajectory.joint_names) {
              if (!current_positions.count(name)) {
                  RCLCPP_ERROR(this->get_logger(), "Missing scan joint state: %s", name.c_str());
                  return false;
              }
              start.positions.push_back(current_positions.at(name));
          }
          start.velocities.assign(target_positions.size(), 0.0);
          start.accelerations.assign(target_positions.size(), 0.0);
          point.velocities = start.velocities;
          point.accelerations = start.accelerations;
          goal_msg.trajectory.points.push_back(start);
          
          point.time_from_start = rclcpp::Duration::from_seconds(duration); 
          goal_msg.trajectory.points.push_back(point);
          
          RCLCPP_INFO(this->get_logger(), "Sending goal: J1 -> %.2f (Duration: %.2fs)", target_val, duration);

          auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
          auto goal_handle_future = trajectory_client_->async_send_goal(goal_msg, send_goal_options);
          
          if (goal_handle_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
             RCLCPP_ERROR(this->get_logger(), "Send goal timed out");
             return false;
          }

          auto goal_handle = goal_handle_future.get();
          if (!goal_handle) {
             RCLCPP_ERROR(this->get_logger(), "Goal rejected");
             return false;
          }

          auto result_future = trajectory_client_->async_get_result(goal_handle);
          
          RCLCPP_INFO(this->get_logger(), "Waiting for execution result...");
          if (result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
              trajectory_client_->async_cancel_goal(goal_handle);
              RCLCPP_ERROR(this->get_logger(), "Execution timed out (Result wait)");
              return false;
          }

          auto wrapped_result = result_future.get();
          if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
               RCLCPP_ERROR(this->get_logger(), "Trajectory CANCELED (Preempted by another goal?)");
               return false;
          }
          if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_ERROR(this->get_logger(), "Scan step %zu failed: action=%d, controller=%d, %s",
                           i+1, (int)wrapped_result.code, wrapped_result.result->error_code,
                           wrapped_result.result->error_string.c_str());
              return false;
          }
          
          RCLCPP_INFO(this->get_logger(), "Step %zu Complete. Pausing...", i+1);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }

      RCLCPP_INFO(this->get_logger(), "Scan Sequence Complete");
      return true;
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_home_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr scan_service_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_, cartesian_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex joint_mutex_;
  std::mutex motion_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_client_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotActionsNode>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  std::thread([&node](){
      std::this_thread::sleep_for(std::chrono::seconds(2));
      node->init_move_group();
  }).detach();

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
