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
  , prev_robot_yaw_(0.0)             // 初始化角度记录
  , return_to_init_retry_count_(0)   // 初始化返回初始位置重试计数
{
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  // 移除progress_timeout参数，不再使用超时逻辑
  this->declare_parameter<float>("stuck_distance_threshold", 0.05);  // 卡住距离检测阈值（米）
  this->declare_parameter<float>("stuck_angle_threshold", 0.17);  // 卡住角度检测阈值（弧度，约10度）
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("stuck_distance_threshold", stuck_distance_threshold_);  // 获取卡住距离检测阈值
  this->get_parameter("stuck_angle_threshold", stuck_angle_threshold_);  // 获取角度阈值
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
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

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
  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
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
    if (goalOnBlacklist(frontier.centroid)) {
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
  // 🎯 核心探索规划函数 - 实现前沿检测和目标选择
  // ⚠️ 注意：每次调用都会重新评估当前情况，可能改变导航目标！

  // 📍 获取当前机器人位姿
  auto pose = costmap_client_.getRobotPose();

  // 🔍 前沿检测 - 寻找已知区域与未知区域的边界
  // search_.searchFrom() 返回按代价排序的前沿列表（每次都会重新计算）
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());

  // 调试输出：显示所有检测到的前沿及其代价
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  // ❌ 无前沿可探索 - 停止探索
  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    stop(true);  // finished_exploring = true
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
                         return goalOnBlacklist(f.centroid);  // 检查是否在黑名单中
                       });

  // ❌ 所有前沿都在黑名单中 - 探索完成
  if (frontier == frontiers.end()) {
    RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
    stop(true);  // 探索完成
    return;
  }

  // ✅ 选择最佳前沿的质心作为导航目标
  geometry_msgs::msg::Point target_position = frontier->centroid;

  // 🔄 智能卡住检测 - 同时检查位置和角度变化

  // 获取当前机器人完整位姿（位置和朝向）
  geometry_msgs::msg::Point current_robot_position = pose.position;
  double current_robot_yaw = tf2::getYaw(pose.orientation);  // 从四元数提取偏航角

  // 检查是否是相同的目标（避免重复导航）
  bool same_goal = same_point(prev_goal_, target_position);

  // 🚫 智能导航决策：只有卡住时才重新规划
  bool robot_is_stuck = false;
  bool first_goal = !has_prev_robot_position_;  // 首次运行标志（在位置记录前计算）

  // 检查机器人是否卡住（位置或角度变化都很小）
  if (has_prev_robot_position_) {
    // 计算位置变化
    double dx = current_robot_position.x - prev_robot_position_.x;
    double dy = current_robot_position.y - prev_robot_position_.y;
    double distance_moved = sqrt(dx * dx + dy * dy);

    // 计算角度变化（考虑角度的周期性，-π到π）
    double angle_diff = current_robot_yaw - prev_robot_yaw_;
    // 规范化角度差到[-π, π]范围
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
    double angle_changed = fabs(angle_diff);

    // 如果位置变化很小且角度变化也很小，增加卡住计数
    if (distance_moved < stuck_distance_threshold_ && angle_changed < stuck_angle_threshold_) {
      stuck_count_++;
      RCLCPP_DEBUG(logger_, "Potential stuck detected (%d/%d): moved %.3fm, rotated %.3f rad",
                   stuck_count_, STUCK_THRESHOLD, distance_moved, angle_changed);
      
      // 只有连续多次检测到卡住才触发重新规划
      if (stuck_count_ >= STUCK_THRESHOLD) {
        RCLCPP_WARN(logger_,
                    "Robot confirmed stuck (moved only %.3fm, rotated only %.3f rad for %d checks), replanning...",
                    distance_moved, angle_changed, stuck_count_);
        frontier_blacklist_.push_back(prev_goal_);  // 将当前目标加入黑名单
        robot_is_stuck = true;  // 标记为卡住状态
        stuck_count_ = 0;  // 重置计数器
      }
    } else {
      // 机器人正常移动，重置计数器
      stuck_count_ = 0;
    }
  }

  // 📍 记录当前规划时的机器人位置和朝向，用于下次比较
  prev_robot_position_ = current_robot_position;
  prev_robot_yaw_ = current_robot_yaw;
  has_prev_robot_position_ = true;

  // 🔄 状态重置
  if (resuming_) {
    resuming_ = false;  // 清除恢复标志
  }

  // 👁️ 检查当前目标是否已被探索（周围不再有未知区域）
  // 检查目标点周围一定范围内是否还有未知区域
  bool current_goal_explored = false;
  if (navigating_) {
    nav2_costmap_2d::Costmap2D* costmap = costmap_client_.getCostmap();
    unsigned int mx, my;
    if (costmap->worldToMap(prev_goal_.x, prev_goal_.y, mx, my)) {
      // 检查目标点周围 5x5 区域（约 0.25m x 0.25m ）是否还有未知点
      constexpr int CHECK_RADIUS = 2;  // 检查半径（单位：栅格）
      bool has_unknown_nearby = false;
      
      int size_x = static_cast<int>(costmap->getSizeInCellsX());
      int size_y = static_cast<int>(costmap->getSizeInCellsY());
      
      for (int dy = -CHECK_RADIUS; dy <= CHECK_RADIUS && !has_unknown_nearby; ++dy) {
        for (int dx = -CHECK_RADIUS; dx <= CHECK_RADIUS && !has_unknown_nearby; ++dx) {
          int nx = static_cast<int>(mx) + dx;
          int ny = static_cast<int>(my) + dy;
          if (nx >= 0 && nx < size_x && ny >= 0 && ny < size_y) {
            unsigned char cost = costmap->getCost(nx, ny);
            if (cost == nav2_costmap_2d::NO_INFORMATION) {
              has_unknown_nearby = true;
            }
          }
        }
      }
      
      // 只有当周围完全没有未知区域时，才认为目标已被探索
      if (!has_unknown_nearby) {
        current_goal_explored = true;
        RCLCPP_INFO(logger_, "Current goal (%.2f, %.2f) area is fully explored, switching to new frontier",
                    prev_goal_.x, prev_goal_.y);
        // 取消当前导航
        move_base_client_->async_cancel_all_goals();
        navigating_ = false;
      }
    }
  }

  // 🎯 导航决策逻辑：
  // 1. 首次启动时发送导航目标
  // 2. 卡住时重新规划
  // 3. 当前目标已被探索（可见），切换到新目标
  // 4. 如果正在导航且没卡住，继续等待
  // 5. 导航完成后发送新目标
  if (first_goal) {
    RCLCPP_INFO(logger_, "First run, sending initial navigation goal to (%.2f, %.2f)",
                target_position.x, target_position.y);
  } else if (robot_is_stuck) {
    RCLCPP_INFO(logger_, "Robot stuck, sending new navigation goal to (%.2f, %.2f)",
                target_position.x, target_position.y);
  } else if (current_goal_explored) {
    // 当前目标已被探索，发送新目标
    RCLCPP_INFO(logger_, "Frontier visible, sending new goal to (%.2f, %.2f)",
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

  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");

  // 🚀 调用Nav2的/navigate_to_pose动作 - 核心导航接口
  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;              // 设置目标位置（前沿质心）
  goal.pose.pose.orientation.w = 1.;                     // 设置朝向（四元数，朝向任意）
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();  // 坐标系（通常是map）
  goal.pose.header.stamp = this->now();                   // 时间戳

  // 配置动作调用选项
  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  // send_goal_options.goal_response_callback =
  // std::bind(&Explore::goal_response_callback, this, _1);
  // send_goal_options.feedback_callback =
  //   std::bind(&Explore::feedback_callback, this, _1, _2);

  // 📋 结果回调函数 - 导航完成后处理
  send_goal_options.result_callback =
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
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
  // 💾 自动保存地图
  RCLCPP_INFO(logger_, "Saving map automatically...");
  
  // 构建保存命令
  // 自动获取 x_bot 功能包的 share 路径
  std::string x_bot_share = ament_index_cpp::get_package_share_directory("x_bot");
  std::string map_dir = x_bot_share + "/maps";
  std::string map_name = "map";
  std::string map_path = map_dir + "/" + map_name;
  
  std::string command = "ros2 run nav2_map_server map_saver_cli -f " + map_path + " &";
  
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
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  // 🎯 导航结果处理回调函数 - 根据导航结果决定下一步行动
  
  // 🔍 检查导航结果状态
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // ✅ 导航成功 - 继续探索下一个前沿
      RCLCPP_INFO(logger_, "[CALLBACK] Goal SUCCEEDED for (%.2f, %.2f)", frontier_goal.x, frontier_goal.y);
      navigating_ = false;
      break;

    case rclcpp_action::ResultCode::ABORTED:
      // ❌ 导航被中止 - 通常是无法到达，将目标加入黑名单
      RCLCPP_INFO(logger_, "[CALLBACK] Goal ABORTED for (%.2f, %.2f)", frontier_goal.x, frontier_goal.y);
      frontier_blacklist_.push_back(frontier_goal);
      navigating_ = false;
      return;

    case rclcpp_action::ResultCode::CANCELED:
      // 🛑 导航被取消 - 可能是我们主动取消（切换目标）
      RCLCPP_INFO(logger_, "[CALLBACK] Goal CANCELED for (%.2f, %.2f) - keeping navigating_=true", frontier_goal.x, frontier_goal.y);
      // 不设置 navigating_ = false
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
  // 🛑 停止探索
  RCLCPP_INFO(logger_, "Exploration stopped.");

  // 取消所有正在进行的导航目标
  move_base_client_->async_cancel_all_goals();

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
