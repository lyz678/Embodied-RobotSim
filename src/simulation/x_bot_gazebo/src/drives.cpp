// Reuse native Gazebo feedback and expose the shared position command
// interface. An elbow effort drive and bounded, symmetric finger drives handle
// physical contact.
#include "x_bot_gazebo/drives.hpp"
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <cmath>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz_ros2_control/gz_system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>
#include <yaml-cpp/yaml.h>

namespace embodied {
class GazeboDriveSystem : public gz_ros2_control::GazeboSimSystemInterface {
  pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> loader_{
      "gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface"};
  std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> native_;
  gz::sim::EntityComponentManager *ecm_ = nullptr;
  rclcpp::Node::SharedPtr debug_node_;
  int64_t last_trace_ = -1;
  std::array<gz::sim::Entity, 2> fingers_{};
  double finger_stiffness_ = 1000., finger_damping_ = 20.,
         finger_effort_ = 100.;
  std::array<double, 8> targets_{};
  std::vector<hardware_interface::CommandInterface> native_commands_;
  std::array<double, 2> finger_integral_{};
  double last_finger_target_ = .04;
  double finger_integral_gain_ = 1000., finger_integral_limit_ = 20.;
  std::array<gz::sim::Entity, 7> arms_{};
  std::array<gz::sim::Entity, 7> arm_links_{};
  std::vector<gz::sim::Entity> end_links_;
  std::array<double, 7> last_target_{};
  std::vector<double> stiffness_, damping_, inertia_;
  double max_effort_ = 300.;
  bool first_write_ = true;
  int64_t last_write_ns_ = -1;
  auto &native() {
    if (!native_)
      native_ = loader_.createSharedInstance("gz_ros2_control/GazeboSimSystem");
    return *native_;
  }

public:
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override {
    auto result = hardware_interface::SystemInterface::on_init(info);
    if (result != hardware_interface::CallbackReturn::SUCCESS)
      return result;
    auto delegated_info = info;
    delegated_info.hardware_plugin_name = "gz_ros2_control/GazeboSimSystem";
    return native().on_init(delegated_info);
  }
  bool initSim(rclcpp::Node::SharedPtr &node,
               std::map<std::string, gz::sim::Entity> &joints,
               const hardware_interface::HardwareInfo &info,
               gz::sim::EntityComponentManager &ecm,
               unsigned int rate) override {
    auto config =
        YAML::LoadFile(ament_index_cpp::get_package_share_directory("x_bot_control") +
                       "/config/gazebo_physics.yaml");
    // Native Gazebo reads this from its ROS node, not hardware XML parameters.
    const double gain = config["position_proportional_gain"].as<double>();
    if (!std::isfinite(gain) || gain <= 0. || gain > 1.)
      throw std::runtime_error("Invalid Gazebo position servo gain");
    const auto configured = node->set_parameter(
        rclcpp::Parameter("position_proportional_gain", gain));
    if (!configured.successful)
      throw std::runtime_error(configured.reason);
    if (!native().initSim(node, joints, info, ecm, rate))
      return false;
    finger_stiffness_ = config["finger_stiffness"].as<double>();
    finger_damping_ = config["finger_damping"].as<double>();
    finger_effort_ = config["gripper_max_effort"].as<double>();
    finger_integral_gain_ = config["finger_integral_gain"].as<double>();
    finger_integral_limit_ = config["finger_integral_limit"].as<double>();
    if (!std::isfinite(finger_stiffness_) || !std::isfinite(finger_damping_) ||
        !std::isfinite(finger_effort_) ||
        !std::isfinite(finger_integral_gain_) ||
        !std::isfinite(finger_integral_limit_) || finger_stiffness_ <= 0 ||
        finger_damping_ < 0 || finger_effort_ <= 0 ||
        finger_integral_gain_ < 0 || finger_integral_limit_ < 0)
      throw std::runtime_error("Invalid Gazebo servo configuration");
    native_commands_ = native().export_command_interfaces();
    ecm_ = &ecm;
    debug_node_ = node;
    fingers_ = {joints.at("fr3_finger_joint1"), joints.at("fr3_finger_joint2")};
    for (size_t i = 0; i < arms_.size(); ++i)
      arms_[i] = joints.at("fr3_joint" + std::to_string(i + 1));
    gz::sim::Model model(
        ecm.Component<gz::sim::components::ParentEntity>(arms_[0])->Data());
    for (size_t i = 0; i < 7; ++i)
      arm_links_[i] = model.LinkByName(ecm, "fr3_link" + std::to_string(i + 1));
    for (const char *name : {"fr3_hand", "fr3_leftfinger", "fr3_rightfinger"})
      end_links_.push_back(model.LinkByName(ecm, name));
    stiffness_ = config["arm_stiffness"].as<std::vector<double>>();
    damping_ = config["arm_damping"].as<std::vector<double>>();
    inertia_ = config["arm_effective_inertia"].as<std::vector<double>>();
    max_effort_ = config["arm_max_effort"].as<double>();
    if (stiffness_.size() != 7 || damping_.size() != 7 ||
        inertia_.size() != 7 || !std::isfinite(max_effort_) ||
        max_effort_ <= 0.)
      throw std::runtime_error("Invalid Gazebo arm drive configuration");
    for (size_t i = 0; i < 7; ++i)
      if (!std::isfinite(stiffness_[i]) || stiffness_[i] <= 0. ||
          !std::isfinite(damping_[i]) || damping_[i] < 0. ||
          !std::isfinite(inertia_[i]) || inertia_[i] <= 0.)
        throw std::runtime_error("Invalid Gazebo arm drive gain");
    auto initial =
        YAML::LoadFile(ament_index_cpp::get_package_share_directory("x_bot_control") +
                       "/config/initial_positions.yaml")["initial_positions"];
    for (size_t i = 0; i < 7; ++i)
      targets_[i] = initial["fr3_joint" + std::to_string(i + 1)].as<double>();
    targets_[7] = initial["fr3_finger_joint1"].as<double>();
    return true;
  }
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override {
    return native().export_state_interfaces();
  }
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> result;
    for (size_t i = 0; i < 7; ++i)
      result.emplace_back("fr3_joint" + std::to_string(i + 1), "position",
                          &targets_[i]);
    result.emplace_back("fr3_finger_joint1", "position", &targets_[7]);
    return result;
  }
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &s) override {
    return native().on_configure(s);
  }
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &s) override {
    return native().on_activate(s);
  }
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &s) override {
    return native().on_deactivate(s);
  }
  hardware_interface::return_type
  perform_command_mode_switch(const std::vector<std::string> &start,
                              const std::vector<std::string> &stop) override {
    return native().perform_command_mode_switch(start, stop);
  }
  hardware_interface::return_type
  read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    return native().read(time, period);
  }
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    // Native velocity servos hold the wrist accurately under changing payload.
    // Joint 2 uses an effort drive to release DART's lower-limit contact.
    for (auto &interface : native_commands_) {
      const auto name = interface.get_prefix_name();
      for (size_t i = 0; i < 7; ++i)
        if (name == "fr3_joint" + std::to_string(i + 1))
          interface.set_value(targets_[i]);
      if (name == "fr3_finger_joint1")
        interface.set_value(targets_[7]);
    }
    const auto result = native().write(time, period);
    if (time.nanoseconds() < last_write_ns_) {
      first_write_ = true;
      finger_integral_.fill(0.);
      last_trace_ = -1;
    }
    last_write_ns_ = time.nanoseconds();
    const double dt = std::clamp(period.seconds(), .001, .05);
    for (size_t i = 1; i < 2; ++i) {
      auto entity = arms_[i];
      auto pos = ecm_->Component<gz::sim::components::JointPosition>(entity);
      auto vel = ecm_->Component<gz::sim::components::JointVelocity>(entity);
      if (!pos || pos->Data().empty() || !vel || vel->Data().empty())
        continue;
      const double target = targets_[i];
      const double target_velocity =
          first_write_
              ? 0.
              : std::clamp((target - last_target_[i]) / dt, -5.26, 5.26);
      last_target_[i] = target;
      // Implicit PD compensation keeps the light wrist stable at a 5 ms tick.
      const double kp = stiffness_.at(i), kd = damping_.at(i);
      double force = implicitTorque(target - pos->Data()[0],
                                    target_velocity - vel->Data()[0], dt, kp,
                                    kd, inertia_.at(i), max_effort_);
      // URDF arm joints rotate about child-link Z at its origin. Sum gravity
      // moments of downstream rigid bodies, including the lumped camera mass.
      const auto pivot = gz::sim::worldPose(arm_links_[i], *ecm_);
      const auto axis = pivot.Rot().RotateVector(gz::math::Vector3d::UnitZ);
      double gravity_torque = 0.;
      auto add_mass = [&](gz::sim::Entity body) {
        auto inertial = ecm_->Component<gz::sim::components::Inertial>(body);
        if (!inertial)
          return;
        auto com =
            (gz::sim::worldPose(body, *ecm_) * inertial->Data().Pose()).Pos();
        auto weight = gz::math::Vector3d(
            0, 0, -9.81 * inertial->Data().MassMatrix().Mass());
        gravity_torque += axis.Dot((com - pivot.Pos()).Cross(weight));
      };
      for (size_t j = i; j < 7; ++j)
        add_mass(arm_links_[j]);
      for (auto body : end_links_)
        add_mass(body);
      force -= gravity_torque;
      ecm_->RemoveComponent<gz::sim::components::JointVelocityCmd>(entity);
      ecm_->SetComponentData<gz::sim::components::JointForceCmd>(
          entity, {std::clamp(force, -max_effort_, max_effort_)});
    }
    first_write_ = false;
    if (std::abs(targets_[7] - last_finger_target_) > 1e-6) {
      finger_integral_.fill(0.);
      last_finger_target_ = targets_[7];
    }
    for (size_t i = 0; i < fingers_.size(); ++i) {
      auto entity = fingers_[i];
      auto pos = ecm_->Component<gz::sim::components::JointPosition>(entity);
      auto vel = ecm_->Component<gz::sim::components::JointVelocity>(entity);
      if (pos && !pos->Data().empty() && vel && !vel->Data().empty()) {
        const double error = targets_[7] - pos->Data()[0];
        finger_integral_[i] =
            std::clamp(finger_integral_[i] + finger_integral_gain_ * error * dt,
                       -finger_integral_limit_, finger_integral_limit_);
        const double force = std::clamp(
            implicitTorque(error, -vel->Data()[0], dt, finger_stiffness_,
                           finger_damping_, .0291, finger_effort_) +
                finger_integral_[i],
            -finger_effort_, finger_effort_);
        if (i == 0 && time.seconds() > last_trace_) {
          last_trace_ = int64_t(time.seconds()) + 1;
          RCLCPP_DEBUG(debug_node_->get_logger(),
                       "DRIVE_TRACE target=%.5f actual=%.5f vel=%.5f "
                       "integral=%.2f force=%.2f",
                       targets_[7], pos->Data()[0], vel->Data()[0],
                       finger_integral_[i], force);
        }
        ecm_->RemoveComponent<gz::sim::components::JointVelocityCmd>(entity);
        ecm_->SetComponentData<gz::sim::components::JointForceCmd>(entity,
                                                                   {force});
      }
    }
    return result;
  }
};
} // namespace embodied
PLUGINLIB_EXPORT_CLASS(embodied::GazeboDriveSystem,
                       gz_ros2_control::GazeboSimSystemInterface)
