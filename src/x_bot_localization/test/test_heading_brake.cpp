#include <gtest/gtest.h>

#include "x_bot_localization/heading_brake.hpp"
using x_bot_localization::headingBrakeRate;
TEST(HeadingBrake, DecisiveLargeTurnAndSymmetricBraking) {
  EXPECT_DOUBLE_EQ(headingBrakeRate(1.5, 0., 1., 2., 2., .2, .12), 1.);
  EXPECT_DOUBLE_EQ(headingBrakeRate(-1.5, 0., 1., 2., 2., .2, .12), -1.);
  auto slow = headingBrakeRate(.3, 1., 1., 2., 2., .2, .12);
  EXPECT_LT(slow, .21);
  EXPECT_DOUBLE_EQ(slow, -headingBrakeRate(-.3, -1., 1., 2., 2., .2, .12));
}
TEST(HeadingBrake, InertiaWithDelayedOdometrySettlesWithoutCrossingTarget) {
  // Command follows the smoother's 2 rad/s^2 limit and an inertial yaw plant;
  // odometry is delayed 0.1 s. Verify both turn directions and no late handoff.
  for (double sign : {-1., 1.}) {
    double error = sign * 1.57, rate = 0., command = 0.;
    double history[10] = {};
    bool settled = false;
    double closest = 10.;
    for (int i = 0; i < 1000; ++i) {
      double measured = history[i % 10];
      history[i % 10] = rate;
      const double target =
          headingBrakeRate(error, measured, 1., 2., 2., .2, .12);
      command = std::clamp(target, command - .02, command + .02);
      rate += (command - rate) * .01 / .12;
      error -= rate * .01;
      closest = std::min(closest, sign * error);
      if (std::abs(error) < .3 && std::abs(measured) < .2) {
        settled = true;
        break;
      }
    }
    EXPECT_TRUE(settled);
    EXPECT_GT(closest, 0.);
    EXPECT_LT(std::abs(rate), .2);
  }
}
