#pragma once
#include <algorithm>
namespace embodied {
inline double implicitTorque(double error, double velocity_error, double dt,
                             double kp, double kd, double inertia,
                             double limit) {
  return std::clamp((kp * error + kd * velocity_error) /
                        (1. + kd * dt / inertia + kp * dt * dt / inertia),
                    -limit, limit);
}
} // namespace embodied
