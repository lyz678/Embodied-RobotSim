#include "x_bot_gazebo/sampling.hpp"
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <embree4/rtcore.h>
#include <geometry_msgs/msg/twist.hpp>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/math/Matrix4.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <unordered_map>
#include <unordered_set>
#include <yaml-cpp/yaml.h>
using namespace gz;
namespace embodied {
struct Collider {
  sim::Entity entity, model;
  unsigned instance_id = RTC_INVALID_GEOMETRY_ID;
  bool enabled = true;
  RTCScene local = nullptr;
  RTCGeometry instance = nullptr;
  math::Pose3d pose;
  math::Vector3d low{1e9, 1e9, 1e9}, high{-1e9, -1e9, -1e9};
};
class GazeboBackend : public sim::System,
                      public sim::ISystemConfigure,
                      public sim::ISystemPreUpdate,
                      public sim::ISystemPostUpdate,
                      public sim::ISystemReset {
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command;
  RTCDevice device = nullptr;
  RTCScene scene = nullptr;
  std::vector<Collider> colliders;
  std::unordered_set<sim::Entity> seen;
  std::unordered_set<unsigned> robot_instances;
  sim::Entity robot = sim::kNullEntity, sensor = sim::kNullEntity,
              hand = sim::kNullEntity;
  std::array<sim::Entity, 4> wheels{};
  std::unordered_map<std::string, sim::Entity> targets;
  std::unordered_map<std::string, math::Pose3d> telemetry_poses;
  std::string robot_name = "x_bot";
  Packet packet;
  int64_t frame_period_ns = period_ns;
  uint64_t index = 0;
  int64_t next = -1, previous = -1, last_telemetry = -1;
  math::Pose3d old_sensor;
  math::Vector3d old_velocity;
  bool initialized = false, ready = false;
  int64_t stable_start = -1;
  double linear = 0, angular = 0, command_time = -1, current_time = 0, v = 0,
         w = 0, yaw_integral = 0, measured_yaw = 0, wheel_yaw = 0;
  YAML::Node motion, physics;
  sim::Entity bottle_body = sim::kNullEntity;
  double setting(const char *name) const { return motion[name].as<double>(); }
  static builtin_interfaces::msg::Time stamp(int64_t ns) {
    builtin_interfaces::msg::Time t;
    t.sec = ns / 1000000000;
    t.nanosec = ns % 1000000000;
    return t;
  }
  sim::Entity modelOf(sim::Entity e,
                      const sim::EntityComponentManager &ecm) const {
    while (e != sim::kNullEntity) {
      if (ecm.Component<sim::components::Model>(e))
        return e;
      auto p = ecm.Component<sim::components::ParentEntity>(e);
      if (!p)
        break;
      e = p->Data();
    }
    return sim::kNullEntity;
  }
  bool robotPart(sim::Entity e, const sim::EntityComponentManager &ecm) const {
    while (e != sim::kNullEntity) {
      if (e == robot)
        return true;
      auto p = ecm.Component<sim::components::ParentEntity>(e);
      if (!p)
        break;
      e = p->Data();
    }
    return false;
  }
  void dampBottle(sim::EntityComponentManager &ecm) {
    // DART ignores SDF velocity_decay; apply physical viscous forces instead.
    if (!ecm.HasEntity(bottle_body)) {
      bottle_body = sim::kNullEntity;
      ecm.Each<sim::components::Model, sim::components::Name>(
          [&](auto e, auto *, auto *name) {
            if (name->Data() == "water_bottle") {
              bottle_body = sim::Model(e).CanonicalLink(ecm);
              sim::Link(bottle_body).EnableVelocityChecks(ecm);
            }
            return true;
          });
    }
    sim::Link body(bottle_body);
    auto linear = body.WorldLinearVelocity(ecm);
    auto angular = body.WorldAngularVelocity(ecm);
    auto inertial = ecm.Component<sim::components::Inertial>(bottle_body);
    if (!linear || !angular || !inertial)
      return;
    const auto rotation =
        (sim::worldPose(bottle_body, ecm) * inertial->Data().Pose()).Rot();
    const auto local_omega = rotation.RotateVectorReverse(*angular);
    const auto torque = rotation.RotateVector(
                            inertial->Data().MassMatrix().Moi() * local_omega) *
                        -physics["bottle_angular_damping"].as<double>();
    body.AddWorldWrench(ecm,
                        *linear *
                            (-inertial->Data().MassMatrix().Mass() *
                             physics["bottle_linear_damping"].as<double>()),
                        torque);
  }
  void discover(sim::EntityComponentManager &ecm) {
    if (robot == sim::kNullEntity)
      ecm.Each<sim::components::Model, sim::components::Name>(
          [&](auto e, auto *, auto *n) {
            if (n->Data() == robot_name)
              robot = e;
            return true;
          });
    if (robot == sim::kNullEntity || sensor != sim::kNullEntity)
      return;
    sim::Model m(robot);
    sensor = m.LinkByName(ecm, "mid360_link");
    hand = m.LinkByName(ecm, "fr3_hand");
    // Fixed links can be preserved by the URDF preserveFixedJoint extension.
    const char *names[] = {"front_left_wheel_joint", "front_right_wheel_joint",
                           "back_left_wheel_joint", "back_right_wheel_joint"};
    for (int i = 0; i < 4; ++i)
      wheels[i] = m.JointByName(ecm, names[i]);
    if (sensor == sim::kNullEntity)
      throw std::runtime_error(
          "MID360 fixed link missing; preserveFixedJoint required");
  }
  void addCollider(sim::Entity e, sim::Entity m, const sdf::Geometry &g,
                   const sim::EntityComponentManager &ecm) {
    std::vector<std::array<float, 3>> vertices;
    std::vector<std::array<unsigned, 3>> faces;
    auto vert = [&](double x, double y, double z) {
      vertices.push_back({float(x), float(y), float(z)});
    };
    auto box = [&](math::Vector3d s) {
      for (int i = 0; i < 8; i++)
        vert((i & 1 ? 1 : -1) * s.X() / 2, (i & 2 ? 1 : -1) * s.Y() / 2,
             (i & 4 ? 1 : -1) * s.Z() / 2);
      unsigned f[][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5},
                         {0, 4, 5}, {0, 5, 1}, {2, 3, 7}, {2, 7, 6},
                         {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
      for (auto &a : f)
        faces.push_back({a[0], a[1], a[2]});
    };
    if (g.Type() == sdf::GeometryType::BOX)
      box(g.BoxShape()->Size());
    else if (g.Type() == sdf::GeometryType::PLANE) {
      auto p = g.PlaneShape();
      math::Quaterniond q;
      q.SetFrom2Axes(math::Vector3d::UnitZ, p->Normal());
      for (auto a : std::array<math::Vector3d, 4>{{{-1000, -1000, 0},
                                                   {1000, -1000, 0},
                                                   {1000, 1000, 0},
                                                   {-1000, 1000, 0}}}) {
        a = q.RotateVector(a);
        vert(a.X(), a.Y(), a.Z());
      }
      faces = {{0, 1, 2}, {0, 2, 3}};
    } else if (g.Type() == sdf::GeometryType::SPHERE ||
               g.Type() == sdf::GeometryType::CYLINDER) {
      bool sphere = g.Type() == sdf::GeometryType::SPHERE;
      double r =
          sphere ? g.SphereShape()->Radius() : g.CylinderShape()->Radius();
      double length = sphere ? 2 * r : g.CylinderShape()->Length();
      int rings = sphere ? 32 : 1;
      for (int j = 0; j <= rings; j++)
        for (int i = 0; i < 96; i++) {
          double a = 2 * M_PI * i / 96, b = M_PI * j / rings;
          double rr = sphere ? r * std::sin(b) : r;
          vert(rr * std::cos(a), rr * std::sin(a),
               sphere ? r * std::cos(b) : (j - .5) * length);
        }
      for (int j = 0; j < rings; j++)
        for (int i = 0; i < 96; i++) {
          unsigned a = j * 96 + i, b = j * 96 + (i + 1) % 96,
                   c = (j + 1) * 96 + i, d = (j + 1) * 96 + (i + 1) % 96;
          faces.push_back({a, b, c});
          faces.push_back({b, d, c});
        }
      if (!sphere) {
        vert(0, 0, -length / 2);
        vert(0, 0, length / 2);
        for (unsigned i = 0; i < 96; i++) {
          faces.push_back({192, i, (i + 1) % 96});
          faces.push_back({193, 96 + (i + 1) % 96, 96 + i});
        }
      }
    } else if (g.Type() == sdf::GeometryType::MESH) {
      auto mesh = g.MeshShape();
      std::string path = mesh->Uri();
      if (path.rfind("file://", 0) == 0)
        path = path.substr(7);
      // Use Gazebo's loader so COLLADA units/node transforms/up-axis behavior
      // exactly matches its physics meshes, including convex decomposition.
      auto mesh_data = sim::loadMesh(*mesh);
      if (!mesh_data)
        throw std::runtime_error("Cannot load collision mesh " + path);
      auto scale = mesh->Scale();
      for (unsigned k = 0; k < mesh_data->SubMeshCount(); k++) {
        auto sub = mesh_data->SubMeshByIndex(k).lock();
        if (!sub)
          continue;
        unsigned base = vertices.size();
        for (unsigned i = 0; i < sub->VertexCount(); i++) {
          auto p = sub->Vertex(i);
          vert(p.X() * scale.X(), p.Y() * scale.Y(), p.Z() * scale.Z());
        }
        for (unsigned i = 0; i + 2 < sub->IndexCount(); i += 3)
          faces.push_back({base + unsigned(sub->Index(i)),
                           base + unsigned(sub->Index(i + 1)),
                           base + unsigned(sub->Index(i + 2))});
      }
    } else
      throw std::runtime_error(
          "Unsupported Gazebo collision geometry for lidar");
    if (vertices.empty() || faces.empty())
      throw std::runtime_error("Empty collision geometry");
    Collider c;
    c.entity = e;
    c.model = m;
    c.local = rtcNewScene(device);
    auto mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    void *vb = rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0,
                                       RTC_FORMAT_FLOAT3, 12, vertices.size());
    std::memcpy(vb, vertices.data(), vertices.size() * 12);
    void *fb = rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0,
                                       RTC_FORMAT_UINT3, 12, faces.size());
    std::memcpy(fb, faces.data(), faces.size() * 12);
    for (auto &p : vertices)
      for (int i = 0; i < 3; i++) {
        c.low[i] = std::min(c.low[i], double(p[i]));
        c.high[i] = std::max(c.high[i], double(p[i]));
      }
    rtcCommitGeometry(mesh);
    rtcAttachGeometry(c.local, mesh);
    rtcReleaseGeometry(mesh);
    rtcCommitScene(c.local);
    c.instance = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE);
    rtcSetGeometryInstancedScene(c.instance, c.local);
    c.instance_id = rtcAttachGeometry(scene, c.instance);
    if (robotPart(e, ecm))
      robot_instances.insert(c.instance_id);
    c.pose = sim::worldPose(e, ecm);
    setTransform(c);
    colliders.push_back(c);
    seen.insert(e);
  }
  void setTransform(Collider &c) {
    math::Matrix4d m(c.pose);
    float t[12];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 4; j++)
        t[i * 4 + j] = m(i, j);
    rtcSetGeometryTransform(c.instance, 0, RTC_FORMAT_FLOAT3X4_ROW_MAJOR, t);
    rtcCommitGeometry(c.instance);
  }
  void updateScene(const sim::EntityComponentManager &ecm) {
    bool changed = false;
    ecm.Each<sim::components::Collision, sim::components::Geometry>(
        [&](auto e, auto *, auto *g) {
          if (!seen.count(e)) {
            addCollider(e, modelOf(e, ecm), g->Data(), ecm);
            changed = true;
          }
          return true;
        });
    for (auto &c : colliders) {
      if (!ecm.Component<sim::components::Collision>(c.entity)) {
        if (c.enabled) {
          rtcDisableGeometry(c.instance);
          c.enabled = false;
          changed = true;
        }
        continue;
      }
      if (!c.enabled) {
        rtcEnableGeometry(c.instance);
        c.enabled = true;
        changed = true;
      }
      auto p = sim::worldPose(c.entity, ecm);
      if (p != c.pose) {
        c.pose = p;
        setTransform(c);
        changed = true;
      }
    }
    if (changed)
      rtcCommitScene(scene);
  }
  void publishCloud() {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp(packet.start);
    msg.header.frame_id = "mid360_link";
    msg.height = 1;
    msg.width = packet.data.size() / 24;
    msg.point_step = 24;
    msg.row_step = msg.width * 24;
    msg.is_dense = true;
    const char *names[] = {"x",           "y",    "z",  "intensity",
                           "offset_time", "line", "tag"};
    int offsets[] = {0, 4, 8, 12, 16, 20, 21};
    int types[] = {7, 7, 7, 7, 6, 2, 2};
    for (int i = 0; i < 7; i++) {
      sensor_msgs::msg::PointField f;
      f.name = names[i];
      f.offset = offsets[i];
      f.datatype = types[i];
      f.count = 1;
      msg.fields.push_back(f);
    }
    msg.data = std::move(packet.data);
    cloud->publish(std::move(msg));
  }
  void publishTelemetry(int64_t ns, const sim::EntityComponentManager &ecm) {
    if (last_telemetry >= 0 && ns - last_telemetry < period_ns)
      return;
    const double telemetry_dt = (ns - last_telemetry) * 1e-9;
    if (last_telemetry < 0 || telemetry_dt <= 0.)
      telemetry_poses.clear();
    last_telemetry = ns;
    targets.clear();
    nlohmann::json objects = nlohmann::json::object();
    ecm.Each<sim::components::Model, sim::components::Name>(
        [&](auto e, auto *, auto *n) {
          std::string name = n->Data();
          if (name == "book" || name == "coffee_mug" || name == "coke_can" ||
              name == "water_bottle" || name == "shoe" ||
              name == "kitchen_table" || name == "trash_bin")
            targets[name] = e;
          return true;
        });
    for (auto &n : std::array<std::string, 3>{
             {"fr3_hand", "fr3_leftfinger", "fr3_rightfinger"}}) {
      auto e = sim::Model(robot).LinkByName(ecm, n);
      if (e != sim::kNullEntity)
        targets[n] = e;
    }
    math::Vector3d bin_lo{1e9, 1e9, 1e9}, bin_hi{-1e9, -1e9, -1e9};
    for (auto &entry : targets) {
      auto pose = sim::worldPose(entry.second, ecm);
      math::Vector3d lo{1e9, 1e9, 1e9}, hi{-1e9, -1e9, -1e9};
      for (auto &c : colliders)
        if (c.enabled && c.model == entry.second)
          for (int k = 0; k < 8; k++) {
            math::Vector3d p{(k & 1) ? c.high.X() : c.low.X(),
                             (k & 2) ? c.high.Y() : c.low.Y(),
                             (k & 4) ? c.high.Z() : c.low.Z()};
            p = c.pose.Pos() + c.pose.Rot().RotateVector(p);
            for (int i = 0; i < 3; i++) {
              lo[i] = std::min(lo[i], p[i]);
              hi[i] = std::max(hi[i], p[i]);
            }
          }
      auto center = lo.X() < 1e8 ? (lo + hi) / 2 : pose.Pos();
      if (entry.first == "trash_bin") {
        bin_lo = lo;
        bin_hi = hi;
      }
      math::Vector3d linear_velocity, angular_velocity;
      auto old = telemetry_poses.find(entry.first);
      if (old != telemetry_poses.end() && telemetry_dt > 0.) {
        linear_velocity = (pose.Pos() - old->second.Pos()) / telemetry_dt;
        auto rotation = pose.Rot() * old->second.Rot().Inverse();
        // Use the shortest quaternion arc for world-frame angular velocity.
        if (rotation.W() < 0.)
          rotation = -rotation;
        const math::Vector3d vector(rotation.X(), rotation.Y(), rotation.Z());
        const double magnitude = vector.Length();
        if (magnitude > 1e-12)
          angular_velocity =
              vector * (2. * std::atan2(magnitude, rotation.W()) /
                        (magnitude * telemetry_dt));
      }
      telemetry_poses[entry.first] = pose;
      auto &p = pose.Pos();
      auto &q = pose.Rot();
      objects[entry.first] = {
          {"position", {p.X(), p.Y(), p.Z()}},
          {"center", {center.X(), center.Y(), center.Z()}},
          {"linear_velocity",
           {linear_velocity.X(), linear_velocity.Y(), linear_velocity.Z()}},
          {"angular_velocity",
           {angular_velocity.X(), angular_velocity.Y(), angular_velocity.Z()}},
          {"orientation_wxyz", {q.W(), q.X(), q.Y(), q.Z()}}};
    }
    std_msgs::msg::String msg;
    msg.data = nlohmann::json{
        {"simulation_time", ns * 1e-9},
        {"frame", "gazebo_world"},
        {"place_bounds",
         {{bin_lo.X(), bin_lo.Y(), bin_lo.Z()},
          {bin_hi.X(), bin_hi.Y(), bin_hi.Z()}}},
        {"objects",
         objects}}.dump();
    telemetry->publish(msg);
    auto p = sim::worldPose(robot, ecm);
    nav_msgs::msg::Odometry od;
    od.header.stamp = stamp(ns);
    od.header.frame_id = "gazebo_world";
    od.child_frame_id = "base_footprint";
    auto old_robot = telemetry_poses.find(robot_name);
    if (old_robot != telemetry_poses.end() && telemetry_dt > 0.) {
      auto velocity = p.Rot().RotateVectorReverse(
          (p.Pos() - old_robot->second.Pos()) / telemetry_dt);
      od.twist.twist.linear.x = velocity.X();
      od.twist.twist.linear.y = velocity.Y();
      od.twist.twist.linear.z = velocity.Z();
      auto rotation = p.Rot() * old_robot->second.Rot().Inverse();
      if (rotation.W() < 0.)
        rotation = -rotation;
      math::Vector3d vector(rotation.X(), rotation.Y(), rotation.Z());
      const double magnitude = vector.Length();
      if (magnitude > 1e-12) {
        auto rate = p.Rot().RotateVectorReverse(
            vector * (2. * std::atan2(magnitude, rotation.W()) /
                      (magnitude * telemetry_dt)));
        od.twist.twist.angular.x = rate.X();
        od.twist.twist.angular.y = rate.Y();
        od.twist.twist.angular.z = rate.Z();
      }
    }
    telemetry_poses[robot_name] = p;
    od.pose.pose.position.x = p.Pos().X();
    od.pose.pose.position.y = p.Pos().Y();
    od.pose.pose.position.z = p.Pos().Z();
    od.pose.pose.orientation.w = p.Rot().W();
    od.pose.pose.orientation.x = p.Rot().X();
    od.pose.pose.orientation.y = p.Rot().Y();
    od.pose.pose.orientation.z = p.Rot().Z();
    truth->publish(od);
  }

public:
  ~GazeboBackend() override {
    for (auto &c : colliders) {
      rtcReleaseGeometry(c.instance);
      rtcReleaseScene(c.local);
    }
    if (scene)
      rtcReleaseScene(scene);
    if (device)
      rtcReleaseDevice(device);
    if (context)
      context->shutdown("Gazebo backend stopped");
  }
  void Configure(const sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &config,
                 sim::EntityComponentManager &, sim::EventManager &) override {
    motion =
        YAML::LoadFile(ament_index_cpp::get_package_share_directory("x_bot") +
                       "/config/base_motion.yaml");
    for (const char *name : {"yaw_integral_release_time", "wheel_yaw_accel"}) {
      if (!std::isfinite(setting(name)) || setting(name) <= 0)
        throw std::runtime_error("Invalid yaw smoothing parameter");
    }
    const auto sensor_config = YAML::LoadFile(
        ament_index_cpp::get_package_share_directory("x_bot") + "/config/mid360.yaml");
    const double rate = sensor_config["publish_hz"].as<double>();
    if (!std::isfinite(rate) || rate < 10 || rate > 40 || rate != std::floor(rate) ||
        200 % static_cast<int>(rate) != 0)
      throw std::runtime_error("MID360 publish_hz must be 10, 20, 25 or 40");
    frame_period_ns = 1000000000 / static_cast<int>(rate);
    physics =
        YAML::LoadFile(ament_index_cpp::get_package_share_directory("x_bot") +
                       "/config/gazebo_physics.yaml");
    if (config && config->HasElement("robot_name"))
      robot_name = config->Get<std::string>("robot_name");
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions opts;
    opts.context(context);
    node = std::make_shared<rclcpp::Node>("gazebo_mid360", opts);
    rclcpp::ExecutorOptions eo;
    eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node);
    cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/x_bot/mid360/points", rclcpp::SensorDataQoS());
    imu = node->create_publisher<sensor_msgs::msg::Imu>("/livox/imu", 10);
    telemetry = node->create_publisher<std_msgs::msg::String>(
        "/simulation/debug/object_states", 10);
    truth = node->create_publisher<nav_msgs::msg::Odometry>(
        "/debug/ground_truth/odom", 10);
    command = node->create_subscription<geometry_msgs::msg::Twist>(
        "/x_bot/cmd_vel_safe", 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          linear = m->linear.x;
          angular = m->angular.z;
          command_time = current_time;
        });
    device = rtcNewDevice(nullptr);
    scene = rtcNewScene(device);
    rtcSetSceneFlags(scene, RTC_SCENE_FLAG_DYNAMIC);
    rtcCommitScene(scene);
  }
  void Reset(const sim::UpdateInfo &, sim::EntityComponentManager &) override {
    packet.reset();
    next = previous = -1;
    index = 0;
    initialized = ready = false;
    stable_start = -1;
    linear = angular = v = w = yaw_integral = measured_yaw = wheel_yaw = 0;
    command_time = -1;
    last_telemetry = -1;
  }
  void PreUpdate(const sim::UpdateInfo &info,
                 sim::EntityComponentManager &ecm) override {
    if (info.paused)
      return;
    dampBottle(ecm);
    discover(ecm);
    if (robot == sim::kNullEntity)
      return;
    current_time = std::chrono::duration<double>(info.simTime).count();
    executor->spin_some();
    double dt =
        std::clamp(std::chrono::duration<double>(info.dt).count(), 0., .05);
    bool live = command_time >= 0 && current_time - command_time <= .5 &&
                ready && std::isfinite(linear) && std::isfinite(angular);
    double demand = 0;
    if (!live || (linear == 0 && angular == 0))
      v = w = yaw_integral = wheel_yaw = 0;
    else {
      double ratio =
          std::max({1.,
                    std::abs(linear) /
                        setting(linear >= 0 ? "max_forward" : "max_reverse"),
                    std::abs(angular) / setting("max_angular")});
      double vl = linear / ratio, wa = angular / ratio, dv = vl - v,
             dw = wa - w;
      bool decelerating = vl * v >= 0 && std::abs(vl) < std::abs(v),
           braking = wa * w < 0 || std::abs(wa) < std::abs(w);
      // Keep steady skid compensation across small planner corrections.
      // Decisive braking unloads continuously; emergency zero resets above.
      bool unloading = std::abs(wa) < .02 || wa * w < 0 ||
                       std::abs(wa) < .5 * std::abs(w) ||
                       (braking && measured_yaw * w > 0 &&
                        std::abs(measured_yaw) > std::abs(w) + .1);
      if (unloading)
        yaw_integral *= std::exp(-dt / setting("yaw_integral_release_time"));
      double fraction = std::min(
          {1.,
           dv ? setting(decelerating ? "linear_decel" : "linear_accel") * dt /
                    std::abs(dv)
              : 1.,
           dw ? setting(braking ? "angular_decel" : "angular_accel") * dt /
                    std::abs(dw)
              : 1.});
      v += fraction * dv;
      w += fraction * dw;
      double error = w - measured_yaw;
      demand = setting("yaw_feed_forward") * w + setting("yaw_kp") * error +
               yaw_integral;
      if (!unloading && std::abs(wa) >= .02 &&
          (std::abs(demand) < setting("max_wheel_yaw_demand") ||
           demand * error < 0))
        yaw_integral = std::clamp(yaw_integral + setting("yaw_ki") * error * dt,
                                  -setting("yaw_integral_limit"),
                                  setting("yaw_integral_limit"));
      demand = std::clamp(setting("yaw_feed_forward") * w +
                              setting("yaw_kp") * error + yaw_integral,
                          -setting("max_wheel_yaw_demand"),
                          setting("max_wheel_yaw_demand"));
      double step = setting("wheel_yaw_accel") * dt;
      demand = std::clamp(demand, wheel_yaw - step, wheel_yaw + step);
      wheel_yaw = demand;
    }
    for (int i = 0; i < 4; i++) {
      if (wheels[i] == sim::kNullEntity)
        throw std::runtime_error("Missing wheel joint");
      double speed = (v + (i % 2 ? 1 : -1) * demand * setting("track") / 2) /
                     setting("radius");
      ecm.SetComponentData<sim::components::JointVelocityCmd>(wheels[i],
                                                              {speed});
    }
  }
  void PostUpdate(const sim::UpdateInfo &info,
                  const sim::EntityComponentManager &ecm) override {
    if (info.paused || sensor == sim::kNullEntity)
      return;
    int64_t ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
            .count();
    if (previous >= 0 && ns <= previous) {
      packet.reset();
      next = -1;
      initialized = ready = false;
      stable_start = -1;
      index = 0;
      // Commands from the previous timeline must never become live again.
      command_time = -1;
      linear = angular = v = w = yaw_integral = measured_yaw = wheel_yaw = 0;
      last_telemetry = -1;
    }
    if (next >= 0 && ns < next)
      return;
    next = ns + step_ns;
    auto pose = sim::worldPose(sensor, ecm);
    double dt = previous >= 0 ? (ns - previous) * 1e-9 : 0;
    previous = ns;
    math::Vector3d velocity, acceleration, gyro;
    if (initialized && dt > 0) {
      velocity = (pose.Pos() - old_sensor.Pos()) / dt;
      acceleration = (velocity - old_velocity) / dt;
      auto dq = old_sensor.Rot().Inverse() * pose.Rot();
      if (dq.W() < 0.)
        dq = -dq;
      math::Vector3d vector(dq.X(), dq.Y(), dq.Z());
      const double magnitude = vector.Length();
      if (magnitude > 1e-12)
        gyro = vector * (2. * std::atan2(magnitude, dq.W()) / (magnitude * dt));
    }
    acceleration = pose.Rot().RotateVectorReverse(acceleration -
                                                  math::Vector3d(0, 0, -9.81));
    bool stationary = initialized && velocity.Length() < .01 &&
                      gyro.Length() < .02 &&
                      std::abs(acceleration.Length() - 9.81) < .5;
    if (!ready) {
      if (stationary) {
        if (stable_start < 0)
          stable_start = ns;
        ready = ns - stable_start >= 500000000;
      } else
        stable_start = -1;
    }
    measured_yaw = gyro.Z();

    old_sensor = pose;
    old_velocity = velocity;
    initialized = true;
    updateScene(ecm);
    publishTelemetry(ns, ecm);
    if (!ready)
      return;
    sensor_msgs::msg::Imu im;
    im.header.stamp = stamp(ns);
    im.header.frame_id = "mid360_imu_link";
    im.orientation_covariance[0] = -1.;
    im.angular_velocity.x = gyro.X();
    im.angular_velocity.y = gyro.Y();
    im.angular_velocity.z = gyro.Z();
    im.linear_acceleration.x = acceleration.X();
    im.linear_acceleration.y = acceleration.Y();
    im.linear_acceleration.z = acceleration.Z();
    imu->publish(im);
    if (packet.start >= 0 && ns - packet.start >= frame_period_ns) {
      publishCloud();
      packet.reset();
    }
    packet.begin(ns);
    for (int i = 0; i < 1000; i++) {
      auto d = direction(index + i);
      auto world = pose.Rot().RotateVector({d[0], d[1], d[2]});
      RTCRayHit hit{};
      hit.ray.org_x = pose.Pos().X();
      hit.ray.org_y = pose.Pos().Y();
      hit.ray.org_z = pose.Pos().Z();
      hit.ray.dir_x = world.X();
      hit.ray.dir_y = world.Y();
      hit.ray.dir_z = world.Z();
      hit.ray.tnear = .1;
      hit.ray.tfar = 40.;
      hit.ray.mask = ~0u;
      hit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
      RTCIntersectArguments args;
      rtcInitIntersectArguments(&args);
      rtcIntersect1(scene, &hit, &args);
      if (hit.hit.geomID != RTC_INVALID_GEOMETRY_ID &&
          !robot_instances.count(hit.hit.instID[0]) && hit.ray.tfar < 40)
        packet.add(d[0] * hit.ray.tfar, d[1] * hit.ray.tfar,
                   d[2] * hit.ray.tfar, (index + i) % 4, ns);
    }
    index += 1000;
  }
};
} // namespace embodied
GZ_ADD_PLUGIN(embodied::GazeboBackend, gz::sim::System,
              embodied::GazeboBackend::ISystemConfigure,
              embodied::GazeboBackend::ISystemPreUpdate,
              embodied::GazeboBackend::ISystemPostUpdate,
              embodied::GazeboBackend::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(embodied::GazeboBackend, "embodied::GazeboBackend")
