// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 Inria

#pragma once

#include <palimpsest/Dictionary.h>

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"

namespace upkie::cpp::interfaces::bullet {

using palimpsest::Dictionary;

//! Properties for robot joints in the Bullet simulation.
struct JointProperties {
  //! Maximum torque, in N⋅m.
  double maximum_torque = 0.0;

  //! Default constructor for properties initialized to default values.
  JointProperties() = default;

  /*! Initialize configurable properties from a dictionary.
   *
   * \param[in] config Configuration dictionary.
   */
  explicit JointProperties(const Dictionary& config) {}
};

}  // namespace upkie::cpp::interfaces::bullet
