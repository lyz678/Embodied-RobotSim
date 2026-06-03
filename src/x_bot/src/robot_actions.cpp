#include <memory>
#include <thread>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <iostream>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
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
      "/arm_command/pose", 10, std::bind(&RobotActionsNode::topic_callback, this, std::placeholders::_1), sub_opt);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&RobotActionsNode::joint_callback, this, std::placeholders::_1), sub_opt);

    status_pub_ = this->create_publisher<std_msgs::msg::String>("/arm_command/status", 10);
    
    // Create planning scene diff publisher for ACM updates
    planning_scene_diff_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 1);
    
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
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    move_group_->setPoseReferenceFrame("odom"); // Ensure odom frame
    
    // Explicitly set the TCP to our new center point
    // This ensures we reach for the ball with the fingers, not the wrist
    move_group_->setEndEffectorLink("fingers_center");
    
    RCLCPP_INFO(this->get_logger(), "MoveGroup Interface READY");
    RCLCPP_INFO(this->get_logger(), "End Effector Link: %s", move_group_->getEndEffectorLink().c_str());
    
    // Initialize PlanningSceneMonitor and configure ACM to allow finger-octomap collisions
    RCLCPP_INFO(this->get_logger(), "Initializing PlanningSceneMonitor...");
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), "robot_description");
    
    if (planning_scene_monitor_) {
      planning_scene_monitor_->startSceneMonitor();
      planning_scene_monitor_->startWorldGeometryMonitor();
      planning_scene_monitor_->startStateMonitor();
      
      // Wait briefly for scene to initialize
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      
      // Modify ACM to allow fingers to collide with octomap
      moveit_msgs::msg::PlanningScene planning_scene_diff;
      {
        planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
        collision_detection::AllowedCollisionMatrix& acm = 
          scene->getAllowedCollisionMatrixNonConst();
        
        std::vector<std::string> finger_links = {
          "fr3_leftfinger", "fr3_rightfinger"
        };
        
        for (const auto& link : finger_links) {
          acm.setEntry("<octomap>", link, true);
          RCLCPP_INFO(this->get_logger(), 
            "[ACM] Allowed collision: %s <-> <octomap>", link.c_str());
        }
        
        // Get planning scene diff message
        scene->getPlanningSceneDiffMsg(planning_scene_diff);
      }
      
      // Publish ACM changes to move_group
      planning_scene_diff.is_diff = true;
      planning_scene_diff_publisher_->publish(planning_scene_diff);
      
      RCLCPP_INFO(this->get_logger(), "Published ACM diff to move_group");
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

  void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!ensure_move_group()) {
        RCLCPP_WARN(this->get_logger(), "MoveGroup not yet initialized or failed!");
        return;
    }

    geometry_msgs::msg::PoseStamped target_pose_odom;
    try {
        if (!tf_buffer_->canTransform("odom", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(1.0))) {
             RCLCPP_WARN(this->get_logger(), "Wait for transform failed");
             return;
        }
        target_pose_odom = tf_buffer_->transform(*msg, "odom");
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform to odom failed: %s", ex.what());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received target pose (transformed to odom): Pos(%.3f, %.3f, %.3f) Ori(%.3f, %.3f, %.3f, %.3f)", 
        target_pose_odom.pose.position.x, target_pose_odom.pose.position.y, target_pose_odom.pose.position.z,
        target_pose_odom.pose.orientation.x, target_pose_odom.pose.orientation.y, 
        target_pose_odom.pose.orientation.z, target_pose_odom.pose.orientation.w);
    
    RCLCPP_INFO(this->get_logger(), "=== Before Motion ===");
    print_current_pose();

    // Pass PoseStamped directly so MoveIt handles frame transform
    move_group_->setPoseTarget(target_pose_odom);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

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

  bool perform_go_home() {
      RCLCPP_INFO(this->get_logger(), "Executing Go Home...");
      
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("x_bot");
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

      move_group_->setJointValueTarget(home_joints);
      move_group_->setGoalTolerance(0.01);
      
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          return (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      } else {
          RCLCPP_ERROR(this->get_logger(), "Planning to home failed");
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
          
          double speed = 2.0; // rad/s
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
          
          double duration = std::max(max_diff / speed, 1.0); // Minimum 1.0s
          
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
              RCLCPP_ERROR(this->get_logger(), "Execution timed out (Result wait)");
              return false;
          }

          auto wrapped_result = result_future.get();
          if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
               RCLCPP_ERROR(this->get_logger(), "Trajectory CANCELED (Preempted by another goal?)");
               return false;
          }
          if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_ERROR(this->get_logger(), "Trajectory failed with code: %d", (int)wrapped_result.code);
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
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_home_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr scan_service_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex joint_mutex_;
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
