// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <map>
#include <string>

#include "upkie/cpp/exceptions/UpkieError.h"

namespace upkie::cpp::actuation {

//! Servo properties.
struct ServoProperties {
  //! Name of the joint the servo acts on.
  std::string joint_name;

  //! Maximum joint torque that the servo can exert, in [N m].
  double maximum_torque;
};

}  // namespace upkie::cpp::actuation
