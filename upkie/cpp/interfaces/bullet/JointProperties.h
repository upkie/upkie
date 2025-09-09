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

  //! Standard deviation of white noise added to applied torques, in N⋅m.
  double torque_control_noise = 0.0;

  //! Default constructor for properties initialized to default values.
  JointProperties() = default;

  /*! Initialize configurable properties from a dictionary.
   *
   * \param[in] config Configuration dictionary.
   */
  explicit JointProperties(const Dictionary& config) {
    torque_control_noise =
        config.get<double>("torque_control_noise", torque_control_noise);
  }

  /*! Update configurable properties from another instance.
   *
   * \param[in] other Properties to update from.
   */
  void update_configurable(const JointProperties& other) noexcept {
    torque_control_noise = other.torque_control_noise;
  }

  //! Reset configurable properties.
  void reset_configurable() noexcept { torque_control_noise = 0.0; }
};

}  // namespace upkie::cpp::interfaces::bullet
