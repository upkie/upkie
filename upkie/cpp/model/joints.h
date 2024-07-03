// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <string>
#include <vector>

#include "upkie/cpp/actuation/ServoLayout.h"

namespace upkie::model {

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

}  // namespace upkie::model
