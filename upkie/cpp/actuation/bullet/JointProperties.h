// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 Inria

#pragma once

#include <palimpsest/Dictionary.h>

#include <string>

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"

namespace upkie::cpp::actuation::bullet {

using palimpsest::Dictionary;

//! Properties for robot joints in the Bullet simulation.
struct JointProperties {
  //! Kinetic friction, in [N m].
  double friction = 0.0;

  //! Maximum torque, in [N m].
  double maximum_torque = 0.0;

  //! Standard deviation of white noise added to applied torques, in [N m].
  double torque_control_noise = 0.0;

  //! Standard deviation of white noise added to measured torques, in [N m].
  double torque_measurement_noise = 0.0;

  //! Default constructor for properties initialized to default values.
  JointProperties() = default;

  /*! Initialize configurable properties from a dictionary.
   *
   * \param[in] props Configuration dictionary.
   */
  explicit JointProperties(const Dictionary& props) {
    friction = props.get<double>("friction", friction);
    torque_control_noise =
        props.get<double>("torque_control_noise", torque_control_noise);
    torque_measurement_noise =
        props.get<double>("torque_measurement_noise", torque_measurement_noise);
  }

  /*! Update configurable properties from another instance.
   *
   * \param[in] other Properties to update from.
   */
  void update_configurable(const JointProperties& other) noexcept {
    friction = other.friction;
    torque_control_noise = other.torque_control_noise;
    torque_measurement_noise = other.torque_measurement_noise;
  }

  //! Reset configurable properties.
  void reset_configurable() noexcept {
    friction = 0.0;
    torque_control_noise = 0.0;
    torque_measurement_noise = 0.0;
  }
};

}  // namespace upkie::cpp::actuation::bullet
