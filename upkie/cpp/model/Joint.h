// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <map>
#include <string>

namespace upkie::cpp::model {

//! Revolute joint properties.
struct Joint {
  //! Maximum joint angle in radians.
  double maximum_position;

  //! Maximum joint torque in [N m].
  double maximum_torque;

  //! Maximum joint velocity in [rad] / [s].
  double maximum_velocity;

  //! Minimum joint angle in radians.
  double minimum_position;

  //! Joint name.
  std::string name;
};

}  // namespace upkie::cpp::model
