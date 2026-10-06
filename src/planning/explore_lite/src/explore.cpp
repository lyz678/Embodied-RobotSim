/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/create_timer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , last_markers_count_(0)
  , has_prev_robot_position_(false)  // 初始化位置记录标志
  , return_to_init_retry_count_(0)   // 初始化返回初始位置重试计数
{
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  progress_watchdog_.timeout = this->declare_parameter<double>("stuck_timeout", 20.0);
  this->declare_parameter<float>("stuck_distance_threshold", 0.05);  // 卡住距离检测阈值（米）
  this->declare_parameter<float>("stuck_angle_threshold", 0.17);  // 卡住角度检测阈值（弧度，约10度）
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<std::string>("map_save_path", "");
  this->declare_parameter<std::string>("map_save_service", "");
  const auto travel_cost = this->declare_parameter<int>("max_travel_cost", 200);
  completion_checks_ = this->declare_parameter<int>("completion_checks", 5);
  frontier_retry_.radius = this->declare_parameter<double>("blacklist_radius", .25);
  frontier_retry_.failure_cooldown = this->declare_parameter<double>("retry_cooldown", 20.);
  frontier_retry_.success_cooldown = this->declare_parameter<double>("observation_wait", 6.);
  const auto attempts = this->declare_parameter<int>("max_goal_attempts", 3);
  if (travel_cost < 0 || travel_cost >= 253 || completion_checks_ < 1 || attempts < 1 ||
      !std::isfinite(frontier_retry_.radius) || frontier_retry_.radius <= 0 ||
      !std::isfinite(frontier_retry_.failure_cooldown) || frontier_retry_.failure_cooldown < 0 ||
      !std::isfinite(frontier_retry_.success_cooldown) || frontier_retry_.success_cooldown < 0) {
    throw std::invalid_argument("Invalid frontier traversal/retry parameters");
  }
  frontier_retry_.max_attempts = static_cast<unsigned>(attempts);


  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("stuck_distance_threshold", stuck_distance_threshold_);  // 获取卡住距离检测阈值
  this->get_parameter("stuck_angle_threshold", stuck_angle_threshold_);  // 获取角度阈值
  progress_watchdog_.distance = stuck_distance_threshold_;
  progress_watchdog_.angle = stuck_angle_threshold_;
  if (!std::isfinite(progress_watchdog_.timeout) || progress_watchdog_.timeout <= 0 ||
      !std::isfinite(progress_watchdog_.distance) || progress_watchdog_.distance <= 0 ||
      !std::isfinite(progress_watchdog_.angle) || progress_watchdog_.angle <= 0)
    throw std::invalid_argument("Invalid exploration progress watchdog parameters");
  // 移除progress_timeout获取
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  // 移除progress_timeout_赋值

  // 🔗 创建Nav2导航动作客户端 - /navigate_to_pose
  // ACTION_NAME = "navigate_to_pose"
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  // 🧠 初始化前沿搜索器 - 核心探索算法
  // FrontierSearch(costmap, potential_scale, gain_scale, min_frontier_size, logger)
  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_, static_cast<unsigned char>(travel_cost));

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  finish_on_observation_ = this->declare_parameter<bool>("finish_on_observation", true);
  const auto observation_topic =
      this->declare_parameter<std::string>("observation_map_topic", "/map");
  observation_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      observation_topic, rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
        observation_map_ = map;
      });

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  // ⏰ 创建探索定时器 - 定期执行探索规划
  // 频率 = planner_frequency_
  exploring_timer_ = rclcpp::create_timer(this, this->get_clock(),
      rclcpp::Duration::from_seconds(1.0 / planner_frequency_),
      [this]() { makePlan(); });
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  // 🎨 前沿可视化函数 - 在RViz中显示探索状态

  // 定义颜色：蓝色(可探索)、红色(黑名单)、绿色(前沿中心)
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0; blue.g = 0; blue.b = 1.0; blue.a = 1.0;  // 蓝色：可探索前沿
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0; red.g = 0; red.b = 0; red.a = 1.0;     // 红色：黑名单前沿
  std_msgs::msg::ColorRGBA green;
  green.r = 0; green.g = 1.0; green.b = 0; green.a = 1.0;  // 绿色：前沿质心

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
#ifdef ELOQUENT
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#elif DASHING
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#else
  m.lifetime = rclcpp::Duration::from_seconds(0);  // foxy onwards
#endif
  // m.lifetime = rclcpp::Duration::from_nanoseconds(0); // suggested in
  // galactic
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    // m.pose.position = {}; // compile warning
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.middle)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  if (exploring_timer_->is_canceled()) {
    return;
  }
  // 🎯 核心探索规划函数 - 实现前沿检测和目标选择
  // ⚠️ 注意：每次调用都会重新评估当前情况，可能改变导航目标！

  // 📍 获取当前机器人位姿
  bool pose_valid = false;
  auto pose = costmap_client_.getRobotPose(&pose_valid);
  if (!pose_valid) { empty_checks_ = 0; return; }

  if (navigating_ && finish_on_observation_ && observation_map_ &&
      observation_map_->header.frame_id == costmap_client_.getGlobalFrameID() &&
      frontierObserved(*observation_map_, active_frontier_points_)) {
    RCLCPP_INFO(logger_,
                "Frontier observed (%zu boundary cells); canceling travel to (%.2f, %.2f)",
                active_frontier_points_.size(), prev_goal_.x, prev_goal_.y);
    ++goal_generation_;  // Ignore the canceled goal's late result/acceptance.
    if (navigation_goal_handle_) move_base_client_->async_cancel_goal(navigation_goal_handle_);
    navigation_goal_handle_.reset();
    frontier_retry_.observed(prev_goal_, this->now().seconds());
    active_frontier_points_.clear();
    navigating_ = false;
    has_prev_robot_position_ = false;
    progress_watchdog_.reset();
  }

  // 🔍 前沿检测 - 寻找已知区域与未知区域的边界
  // search_.searchFrom() 返回按代价排序的前沿列表（每次都会重新计算）
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());

  // 调试输出：显示所有检测到的前沿及其代价
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    if (navigating_) { empty_checks_ = 0; return; }
    if (++empty_checks_ < completion_checks_) {
      RCLCPP_INFO(logger_, "No frontier this tick (%d/%d); waiting for map observations",
                  empty_checks_, completion_checks_);
      return;
    }
    RCLCPP_INFO(logger_, "No reachable frontiers after %d checks; finishing exploration", empty_checks_);
    stop(true);
    return;
  }

  // 📊 可视化前沿（可选）
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // 🎯 选择非黑名单前沿 - 智能避开失败的目标
  // std::find_if_not 返回第一个不满足条件的元素（不在黑名单中）
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.middle);  // 检查是否在黑名单中
                       });

  if (frontier == frontiers.end()) {
    if (navigating_ || frontier_retry_.pending(this->now().seconds())) {
      empty_checks_ = 0;
      RCLCPP_INFO_THROTTLE(logger_, *get_clock(), 5000,
                          "Remaining frontiers are cooling down; exploration continues");
      return;
    }
    if (++empty_checks_ >= completion_checks_) {
      RCLCPP_WARN(logger_, "Remaining frontiers exhausted %u attempts; finishing with unresolved areas",
                  frontier_retry_.max_attempts);
      stop(true);
    }
    return;
  }
  empty_checks_ = 0;
  geometry_msgs::msg::Point target_position = frontier->middle;

  // 🔄 智能卡住检测 - 同时检查位置和角度变化

  // 获取当前机器人完整位姿（位置和朝向）
  geometry_msgs::msg::Point current_robot_position = pose.position;
  double current_robot_yaw = tf2::getYaw(pose.orientation);  // 从四元数提取偏航角

  // 检查是否是相同的目标（避免重复导航）
  bool same_goal = same_point(prev_goal_, target_position);

  // 🚫 智能导航决策：只有卡住时才重新规划
  bool robot_is_stuck = false;
  bool first_goal = !has_prev_robot_position_;  // 首次运行标志（在位置记录前计算）

  // Let Nav2 complete its recovery sequence; slow cumulative motion is progress.
  if (!navigating_ || first_goal) progress_watchdog_.reset();
  if (navigating_ && progress_watchdog_.stalled(
        current_robot_position.x, current_robot_position.y, current_robot_yaw,
        this->now().seconds())) {
    RCLCPP_WARN(logger_, "No cumulative motion progress for %.1f simulation seconds; changing frontier",
                progress_watchdog_.timeout);
    frontier_retry_.record(prev_goal_, this->now().seconds(), false);
    robot_is_stuck = true;
    progress_watchdog_.reset();
  }

  has_prev_robot_position_ = true;

  // 🔄 状态重置
  if (resuming_) {
    resuming_ = false;  // 清除恢复标志
  }

  // Hold this goal while its original unknown boundary still needs observation.

  if (robot_is_stuck) {
    // The blacklist changed after selecting the candidate above.
    frontier = std::find_if_not(frontiers.begin(), frontiers.end(),
        [this](const frontier_exploration::Frontier& f) {
          return goalOnBlacklist(f.middle);
        });
    if (frontier == frontiers.end()) {
      ++goal_generation_;
      if (navigation_goal_handle_) move_base_client_->async_cancel_goal(navigation_goal_handle_);
      navigation_goal_handle_.reset();
      navigating_ = false;
      has_prev_robot_position_ = false;
      return;
    }
    target_position = frontier->middle;
  }

  // 🎯 导航决策逻辑：
  // 1. 首次启动时发送导航目标
  // 2. 卡住时重新规划
  // 3. 如果正在导航且没卡住，继续等待
  // 4. 导航完成后发送新目标
  if (first_goal) {
    RCLCPP_INFO(logger_, "First run, sending initial navigation goal to (%.2f, %.2f)",
                target_position.x, target_position.y);
  } else if (robot_is_stuck) {
    RCLCPP_INFO(logger_, "Robot stuck, sending new navigation goal to (%.2f, %.2f)",
                target_position.x, target_position.y);
  } else if (navigating_) {
    // 正在导航中，等待当前导航完成
    RCLCPP_DEBUG(logger_, "Navigation in progress, waiting for current goal to complete");
    return;
  } else if (same_goal) {
    // 目标相同（导航完成但还没有新前沿）
    RCLCPP_DEBUG(logger_, "Same goal, no new frontier available");
    return;
  } else {
    // 导航已完成，发送新目标
    RCLCPP_INFO(logger_, "Navigation completed, sending new goal to (%.2f, %.2f)",
                target_position.x, target_position.y);
  }

  // 更新目标历史（只有在真正要发送新导航时才更新）
  prev_goal_ = target_position;
  active_frontier_points_ = frontier->points;

  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");

  // 🚀 调用Nav2的/navigate_to_pose动作 - 核心导航接口
  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;              // 设置目标位置（可达观察点）
  const auto boundary = std::min_element(frontier->points.begin(), frontier->points.end(),
      [&target_position](const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
        return std::hypot(a.x-target_position.x, a.y-target_position.y) <
               std::hypot(b.x-target_position.x, b.y-target_position.y);
      });
  const double target_yaw = std::atan2(boundary->y - target_position.y,
                                      boundary->x - target_position.x);
  goal.pose.pose.orientation.z = std::sin(target_yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(target_yaw / 2.0);
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();  // 坐标系（通常是map）
  goal.pose.header.stamp = this->now();                   // 时间戳

  // 配置动作调用选项
  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  const auto generation = ++goal_generation_;
  send_goal_options.goal_response_callback =
      [this, generation, target_position](NavigationGoalHandle::SharedPtr handle) {
        if (generation != goal_generation_) {
          if (handle) {
            move_base_client_->async_cancel_goal(handle);
          }
          return;
        }
        navigation_goal_handle_ = handle;
        if (!handle) {
          RCLCPP_WARN(logger_, "Navigation goal rejected; waiting for retry cooldown");
          frontier_retry_.record(target_position, this->now().seconds(), false);
          navigating_ = false;
          has_prev_robot_position_ = false;
        }
      };
  // send_goal_options.goal_response_callback =
  // std::bind(&Explore::goal_response_callback, this, _1);
  // send_goal_options.feedback_callback =
  //   std::bind(&Explore::feedback_callback, this, _1, _2);

  // 📋 结果回调函数 - 导航完成后处理
  send_goal_options.result_callback =
      [this, generation,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        if (generation == goal_generation_) {
          navigation_goal_handle_.reset();
          reachedGoal(result, target_position);
        }
      };

  // 🎯 异步发送导航目标到Nav2
  navigating_ = true;  // 标记开始导航
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "========================================");
  RCLCPP_INFO(logger_, "🏠 Returning to initial pose (Attempt %d/%d)", 
              return_to_init_retry_count_ + 1, MAX_RETURN_RETRIES);
  RCLCPP_INFO(logger_, "Initial position: (%.2f, %.2f)", 
              initial_pose_.position.x, initial_pose_.position.y);
  RCLCPP_INFO(logger_, "========================================");

  // 🏠 探索完成后返回初始位置 - 同样调用Nav2导航动作
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;       // 初始位置
  goal.pose.pose.orientation = initial_pose_.orientation; // 初始朝向
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  // 📋 设置结果回调函数 - 处理返回结果
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        handleReturnToInitialResult(result);
      };

  // 🚀 发送返回初始位置的导航目标
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::handleReturnToInitialResult(const NavigationGoalHandle::WrappedResult& result)
{
  // 🎯 处理返回初始位置的导航结果
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // ✅ 成功返回初始位置
      RCLCPP_INFO(logger_, "========================================");
      RCLCPP_INFO(logger_, "✅ Successfully returned to initial pose!");
      RCLCPP_INFO(logger_, "========================================");
      return_to_init_retry_count_ = 0;  // 重置重试计数
      break;

    case rclcpp_action::ResultCode::ABORTED:
      // ❌ 导航被中止 - 尝试重试
      RCLCPP_WARN(logger_, "⚠️  Failed to return to initial pose (ABORTED)");
      
      if (return_to_init_retry_count_ < MAX_RETURN_RETRIES) {
        return_to_init_retry_count_++;
        RCLCPP_WARN(logger_, "🔄 Retrying... (Attempt %d/%d)", 
                    return_to_init_retry_count_ + 1, MAX_RETURN_RETRIES);
        
        // 延迟后重试
        std::this_thread::sleep_for(std::chrono::seconds(2));
        returnToInitialPose();
      } else {
        RCLCPP_ERROR(logger_, "========================================");
        RCLCPP_ERROR(logger_, "❌ Failed to return to initial pose after %d attempts", MAX_RETURN_RETRIES);
        RCLCPP_ERROR(logger_, "Robot may be stuck or path is blocked");
        RCLCPP_ERROR(logger_, "========================================");
        return_to_init_retry_count_ = 0;  // 重置计数器
      }
      break;

    case rclcpp_action::ResultCode::CANCELED:
      // 🛑 导航被取消
      RCLCPP_WARN(logger_, "⚠️  Return to initial pose was canceled");
      return_to_init_retry_count_ = 0;
      break;

    default:
      // ❓ 未知结果
      RCLCPP_WARN(logger_, "⚠️  Unknown result code from return to initial pose navigation");
      return_to_init_retry_count_ = 0;
      break;
  }
}

void Explore::saveMap()
{
  // 💾 通过公共建图服务保存入口指定的完整地图包
  RCLCPP_INFO(logger_, "Saving map automatically...");

  const auto service = this->get_parameter("map_save_service").as_string();
  if (!service.empty()) {
    auto client = this->create_client<std_srvs::srv::Trigger>(service);
    if (!client->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(logger_, "Map save service unavailable: %s", service.c_str());
      return;
    }
    // Save the complete paired PCD/2D bundle through its owning node.
    // Keep the client alive until its response without blocking navigation.
    client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
        [client, logger = logger_](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          const auto result = future.get();
          if (result->success) {
            RCLCPP_INFO(logger, "Complete map bundle saved: %s", result->message.c_str());
          } else {
            RCLCPP_ERROR(logger, "Map save failed: %s", result->message.c_str());
          }
        });
    return;
  }

  std::string map_path;
  this->get_parameter("map_save_path", map_path);

  if (map_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Set map_save_service or an explicit map_save_path before saving");
    return;
  }

  std::string command =
      "ros2 run nav2_map_server map_saver_cli -f " + map_path + " &";

  RCLCPP_INFO(logger_, "Saving map to: %s", map_path.c_str());
  
  // 执行保存命令
  int result = system(command.c_str());
  
  if (result == 0) {
    RCLCPP_INFO(logger_, "========================================");
    RCLCPP_INFO(logger_, "✅ Map saved successfully!");
    RCLCPP_INFO(logger_, "📁 Map file: %s.pgm", map_path.c_str());
    RCLCPP_INFO(logger_, "📁 Config file: %s.yaml", map_path.c_str());
    RCLCPP_INFO(logger_, "========================================");
  } else {
    RCLCPP_ERROR(logger_, "❌ Failed to save map (error code: %d)", result);
  }
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  return frontier_retry_.blocked(goal, this->now().seconds());
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  active_frontier_points_.clear();
  // 🎯 导航结果处理回调函数 - 根据导航结果决定下一步行动
  
  // 🔍 检查导航结果状态
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // ✅ 导航成功 - 继续探索下一个前沿
      RCLCPP_INFO(logger_, "[CALLBACK] Goal SUCCEEDED for (%.2f, %.2f)", frontier_goal.x, frontier_goal.y);
      frontier_retry_.record(frontier_goal, this->now().seconds(), true);
      navigating_ = false;
      has_prev_robot_position_ = false;
      progress_watchdog_.reset();
      break;

    case rclcpp_action::ResultCode::ABORTED:
      // ❌ 导航被中止 - 通常是无法到达，将目标加入黑名单
      RCLCPP_INFO(logger_, "[CALLBACK] Goal ABORTED for (%.2f, %.2f)", frontier_goal.x, frontier_goal.y);
      frontier_retry_.record(frontier_goal, this->now().seconds(), false);
      navigating_ = false;
      has_prev_robot_position_ = false;
      progress_watchdog_.reset();
      return;

    case rclcpp_action::ResultCode::CANCELED:
      // 🛑 导航被取消 - 可能是我们主动取消（切换目标）
      RCLCPP_INFO(logger_, "[CALLBACK] Goal CANCELED for (%.2f, %.2f); retrying on next tick", frontier_goal.x, frontier_goal.y);
      navigating_ = false;
      has_prev_robot_position_ = false;
      return;

    default:
      RCLCPP_WARN(logger_, "[CALLBACK] Unknown result code: %d", static_cast<int>(result.code));
      navigating_ = false;
      break;
  }

  // 🔄 立即寻找新目标（无论规划频率如何）
  // 注意：为了防止死锁，这里直接调用makePlan()而不是使用定时器
  // ROS2的单线程执行器特性使得这里不需要额外的定时器机制
  makePlan();
}

void Explore::start()
{
  // 🚀 启动探索 - 记录状态
  RCLCPP_INFO(logger_, "Exploration started.");
}

void Explore::stop(bool finished_exploring)
{
  ++goal_generation_;  // Ignore late results from the previous session/goal.
  active_frontier_points_.clear();
  navigating_ = false;
  // 🛑 停止探索
  RCLCPP_INFO(logger_, "Exploration stopped.");

  // Cancel only this exploration goal; a return/manual goal may start next.
  if (navigation_goal_handle_) {
    move_base_client_->async_cancel_goal(navigation_goal_handle_);
    navigation_goal_handle_.reset();
  }

  // 停止探索定时器
  exploring_timer_->cancel();

  // 💾 如果探索完成，自动保存地图
  if (finished_exploring) {
    RCLCPP_INFO(logger_, "========================================");
    RCLCPP_INFO(logger_, "🎉 Exploration completed!");
    RCLCPP_INFO(logger_, "========================================");
    
    // 自动保存地图
    saveMap();
  }

  // 🔄 如果配置了返回初始位置且探索完成，则返回起点
  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  has_prev_robot_position_ = false;
  progress_watchdog_.reset();
  // ▶️ 恢复探索
  resuming_ = true;  // 设置恢复标志（影响进度检查）
  RCLCPP_INFO(logger_, "Exploration resuming.");

  // 重新激活定时器
  exploring_timer_->reset();

  // 立即开始规划（恢复探索）
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
