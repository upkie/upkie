// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#ifndef ROBOT_SIMULATOR_CLIENT_API_H
#define ROBOT_SIMULATOR_CLIENT_API_H

#include <Eigen/Dense>

#include "Bullet3Common/b3Logging.h"
#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "SharedMemory/PhysicsClientC_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"

//! Structure which contains the new link properties
struct RobotSimulatorChangeDynamicsArgs {
  // The mass of the robot link
  double m_mass;
  // The friction in the lateral direction
  double m_lateralFriction;
  // The friction during rotational motion
  double m_spinningFriction;
  // The friction while rolling
  double m_rollingFriction;
  // The coefficient of restitution
  double m_restitution;
  // The damping factor for linear motion
  double m_linearDamping;
  // The damping factor for angular motion
  double m_angularDamping;
  // The stiffness of contact between objects
  double m_contactStiffness;
  // The damping factor during collisions
  double m_contactDamping;
  // Specifies whether friction is applied at a specific anchor point
  int m_frictionAnchor;
  // The activation state of the link
  int m_activationState;
  // The local inertia tensor diagonal components
  double m_localInertiaDiagonal[3];

  RobotSimulatorChangeDynamicsArgs()
      : m_mass(-1),
        m_lateralFriction(-1),
        m_spinningFriction(-1),
        m_rollingFriction(-1),
        m_restitution(-1),
        m_linearDamping(-1),
        m_angularDamping(-1),
        m_contactStiffness(-1),
        m_contactDamping(-1),
        m_frictionAnchor(-1),
        m_activationState(-1),
        m_localInertiaDiagonal{0, 0, 0} {}
};

//! Child class to enable modification of inertia matrices
class RobotSimulatorClientAPI : public b3RobotSimulatorClientAPI {
 public:
  /* Modifies the properties of a link
   *
   * This is a modification of the method changeDynamics of
   * b3RobotSimulatorClientAPI to enable the modification of the inertia
   * matrices.
   * The usage is identical.
   * \param[in] bodyUniqueId The body which links will be modified
   * \param[in] linkIndex The link id
   * \param[in] args Structure which contains the values to modify
   */
  bool changeDynamics(int bodyUniqueId, int linkIndex,
                      RobotSimulatorChangeDynamicsArgs& args);
};

#endif  // ROBOT_SIMULATOR_CLIENT_API_H
