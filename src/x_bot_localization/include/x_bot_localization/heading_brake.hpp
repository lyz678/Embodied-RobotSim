#pragma once
#include <algorithm>
#include <cmath>
namespace x_bot_localization {
// Brake for localization/actuation latency before releasing heading control.
inline double headingBrakeRate(double error, double measured_rate, double peak,
                               double deceleration, double gain, double latency,
                               double settle_angle) {
  const double direction = error < 0. ? -1. : 1.;
  const double remaining =
      std::max(0., std::abs(error) - direction * measured_rate * latency);
  const double stop_limit =
      std::sqrt(2. * deceleration * std::max(0., remaining - settle_angle));
  return direction * std::min({peak, gain * remaining, stop_limit});
}
}  // namespace x_bot_localization
