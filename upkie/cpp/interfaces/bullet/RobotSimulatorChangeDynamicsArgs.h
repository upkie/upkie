// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

namespace upkie::cpp::interfaces::bullet {

//! Structure which contains the new link properties
struct RobotSimulatorChangeDynamicsArgs {
  //! The mass of the robot link (in kg)
  double m_mass;

  //! The friction in the lateral direction
  double m_lateralFriction;

  //! The friction during rotational motion
  double m_spinningFriction;

  //! The friction while rolling
  double m_rollingFriction;

  //! The coefficient of restitution
  double m_restitution;

  //! The damping factor for linear motion
  double m_linearDamping;

  //! The damping factor for angular motion
  double m_angularDamping;

  //! The stiffness of contact between objects
  double m_contactStiffness;

  //! The damping factor during collisions
  double m_contactDamping;

  //! Specifies whether friction is applied at a specific anchor point
  int m_frictionAnchor;

  //! The activation state of the link
  int m_activationState;

  //! The local inertia tensor diagonal components
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

}  // namespace upkie::cpp::interfaces::bullet
