#include "x_bot_gazebo/drives.hpp"
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
int main(int argc, char **argv) {
  if (argc != 2)
    throw std::runtime_error("Expected Gazebo drive configuration");
  auto config = YAML::LoadFile(argv[1]);
  auto kp = config["arm_stiffness"][1].as<double>();
  auto kd = config["arm_damping"][1].as<double>();
  auto estimate = config["arm_effective_inertia"][1].as<double>();
  double position = -1.8361, velocity = 0.;
  // Force-driven elbow with friction, a hard stop and its physical speed limit.
  // Reversing from the stop must converge instead of alternating at that limit.
  for (double goal : {.4, -1.8361, .2}) {
    for (int tick = 0; tick < 800; ++tick) {
      double torque = embodied::implicitTorque(
          goal - position, -velocity, .005, kp, kd, estimate,
          config["arm_max_effort"].as<double>());
      if (std::abs(velocity) > 1e-5)
        torque -= std::copysign(.2, velocity);
      else if (std::abs(torque) <= .2)
        torque = 0.;
      velocity = std::clamp(velocity + torque / 1.0 * .005, -2.62, 2.62);
      position += velocity * .005;
      if (position < -1.8361) {
        position = -1.8361;
        velocity = std::max(velocity, 0.);
      }
    }
    if (std::abs(position - goal) > .03 || std::abs(velocity) > .05)
      throw std::runtime_error("Elbow did not settle after reversal");
  }
}
