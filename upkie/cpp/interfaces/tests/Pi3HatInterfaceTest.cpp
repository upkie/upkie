// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "upkie/cpp/actuation/Pi3HatInterface.h"

namespace upkie::cpp::actuation {

TEST(Pi3HatInterface, Create) {
  ServoLayout layout;
  layout.add_servo(1, 1, "right_hip");
  layout.add_servo(2, 1, "right_knee");
  layout.add_servo(3, 1, "right_wheel");
  layout.add_servo(4, 2, "left_hip");
  layout.add_servo(5, 2, "left_knee");
  layout.add_servo(6, 2, "left_wheel");

  Pi3Hat::Configuration pi3hat_config;
  pi3hat_config.attitude_rate_hz = 1000u;
  pi3hat_config.mounting_deg.pitch = 0.;
  pi3hat_config.mounting_deg.roll = 0.;
  pi3hat_config.mounting_deg.yaw = 0.;

  constexpr int kCanCpu = 2;
  Pi3HatInterface interface(layout, kCanCpu, pi3hat_config);
}

}  // namespace upkie::cpp::actuation
