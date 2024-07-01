// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 Inria

#pragma once

#include <string>

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"

namespace upkie::cpp::actuation {

//! Properties for robot joints in the Bullet simulation.
struct BulletJointProperties {
  //! Kinetic friction, in N.m
  double friction = 0.0;

  //! Maximum torque, in N.m
  double maximum_torque = 0.0;
};

}  // namespace upkie::cpp::actuation
