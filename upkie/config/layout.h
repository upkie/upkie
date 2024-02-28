// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <vulp/actuation/ServoLayout.h>

#include <string>
#include <vector>

namespace upkie::config {

constexpr int kLeftBusID = 1;   // JC1
constexpr int kRightBusID = 3;  // JC3

/*! Get Upkie's servo layout.
 *
 * \return Upkie's servo layout.
 */
inline const vulp::actuation::ServoLayout servo_layout() noexcept {
  vulp::actuation::ServoLayout layout;
  layout.add_servo(1, kLeftBusID, "left_hip");
  layout.add_servo(2, kLeftBusID, "left_knee");
  layout.add_servo(3, kLeftBusID, "left_wheel");
  layout.add_servo(4, kRightBusID, "right_hip");
  layout.add_servo(5, kRightBusID, "right_knee");
  layout.add_servo(6, kRightBusID, "right_wheel");
  return layout;
}

/*! Get list of upper leg joints, i.e. hips and knees.
 *
 * \return Vector of upper leg joint names.
 */
inline const std::vector<std::string> upper_leg_joints() noexcept {
  return {"left_hip", "left_knee", "right_hip", "right_knee"};
}

/*! Get list of wheel joints.
 *
 * \return Vector of wheel joint names.
 */
inline const std::vector<std::string> wheel_joints() noexcept {
  return {"left_wheel", "right_wheel"};
}

}  // namespace upkie::config
