// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <vulp/actuation/ServoLayout.h>

#include <string>
#include <vector>

namespace upkie::config {

/*! Get Upkie's servo layout.
 *
 * \return Upkie's servo layout.
 */
inline const vulp::actuation::ServoLayout servo_layout() noexcept {
  vulp::actuation::ServoLayout layout;
  layout.add_servo(1, 1, "left_hip");
  layout.add_servo(2, 1, "left_knee");
  layout.add_servo(3, 1, "left_wheel");
  layout.add_servo(4, 2, "right_hip");
  layout.add_servo(5, 2, "right_knee");
  layout.add_servo(6, 2, "right_wheel");
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
