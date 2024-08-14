// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 Inria

#pragma once

#include <palimpsest/Dictionary.h>

#include <string>

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"

namespace upkie::cpp::actuation {

using palimpsest::Dictionary;

//! Properties for robot joints in the Bullet simulation.
struct BulletJointProperties {
  //! Kinetic friction, in [N m].
  double friction = 0.0;

  //! Maximum torque, in [N m].
  double maximum_torque = 0.0;

  //! Standard deviation of white noise added to joint torques, in [N m].
  double torque_noise = 0.0;

  //! Default constructor for properties initialized to default values.
  BulletJointProperties() = default;

  /*! Initialize configurable properties from a dictionary.
   *
   * \param[in] props Configuration dictionary.
   */
  explicit BulletJointProperties(const Dictionary& props) {
    friction = props.get<double>("friction", friction);
    torque_noise = props.get<double>("torque_noise", torque_noise);
  }

  /*! Update configurable properties from another instance.
   *
   * \param[in] other Properties to update from.
   */
  void update_configurable(const BulletJointProperties& other) noexcept {
    friction = other.friction;
    torque_noise = other.torque_noise;
  }

  //! Reset configurable properties.
  void reset_configurable() noexcept {
    friction = 0.0;
    torque_noise = 0.0;
  }
};

}  // namespace upkie::cpp::actuation
