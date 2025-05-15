// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Inria

#include "upkie/cpp/controllers/WheelStopper.h"

namespace upkie::cpp::controllers {

WheelStopper::WheelStopper() {}

void WheelStopper::reset(const Dictionary& config) {}

void WheelStopper::read(const Dictionary& observation,
                        const Dictionary& action) {}

void WheelStopper::write(Dictionary& action) {
  for (const auto& wheel : {"left_wheel", "right_wheel"}) {
    auto& servo_action = action("servo")(wheel);
    servo_action("feedforward_torque") = 0.0;  // [N.m]
    servo_action("position") = std::numeric_limits<double>::quiet_NaN();
    servo_action("velocity") = 0.0;  // [rad/s]
  }
}

}  // namespace upkie::cpp::controllers
