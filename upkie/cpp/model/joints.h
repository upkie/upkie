// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <limits>
#include <string>
#include <vector>

#include "upkie/cpp/exceptions/UpkieError.h"
#include "upkie/cpp/model/Joint.h"

//! Robot model properties.
namespace upkie::cpp::model {

/*! Get joint properties from joint name.
 *
 * \note Joint limits are documented with the `upkie_description` package, on
 * [this page](https://github.com/upkie/upkie_description/wiki/Joint-limits).
 *
 * \param[in] joint_name Joint name.
 * \return Joint properties.
 * \throw UpkieError If there is no joint with this name in the model.
 */
inline Joint get_joint(const std::string& joint_name) {
  if (joint_name == "left_hip" || joint_name == "right_hip") {
    Joint hip;
    hip.maximum_position = 1.26;   // [rad]
    hip.maximum_torque = 16.0;     // [N m]
    hip.maximum_velocity = 28.8;   // [rad] / [s]
    hip.minimum_position = -1.26;  // [rad]
    hip.name = joint_name;
    return hip;
  } else if (joint_name == "left_knee" || joint_name == "right_knee") {
    Joint knee;
    knee.maximum_position = 2.51;   // [rad]
    knee.maximum_torque = 16.0;     // [N m]
    knee.maximum_velocity = 28.8;   // [rad] / [s]
    knee.minimum_position = -2.51;  // [rad]
    knee.name = joint_name;
    return knee;
  } else if (joint_name == "left_wheel" || joint_name == "right_wheel") {
    const double infinity = std::numeric_limits<double>::infinity();
    Joint wheel;
    wheel.maximum_position = +infinity;
    wheel.minimum_position = -infinity;
    wheel.maximum_velocity = 111.0;  // [rad] / [s]
    wheel.maximum_torque = 1.7;      // [N m]
    wheel.name = joint_name;
    return wheel;
  } else {
    std::string error = "[get_joint] Unknown joint ";
    throw exceptions::UpkieError(error + "\"" + joint_name + "\"");
  }
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

}  // namespace upkie::cpp::model
