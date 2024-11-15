// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#include "RobotSimulator.h"
#include <iostream>

bool RobotSimulatorClientAPI::changeDynamics(
  int bodyUniqueId,
  int linkIndex,
  struct RobotSimulatorChangeDynamicsArgs& args
  ) {
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
  b3Warning("Not connected to physics server.");
    return false;
  }
  b3SharedMemoryCommandHandle command = b3InitChangeDynamicsInfo(sm);
  b3SharedMemoryStatusHandle statusHandle;

  if (args.m_activationState >= 0) {
    b3ChangeDynamicsInfoSetActivationState(
    command,
    bodyUniqueId,
    args.m_activationState);
  }
  if (args.m_mass >= 0) {
  b3ChangeDynamicsInfoSetMass(command, bodyUniqueId, linkIndex, args.m_mass);
  }
    if (!(args.m_localInertiaDiagonal[0] == 0
  && args.m_localInertiaDiagonal[1] == 0
  && args.m_localInertiaDiagonal[2] == 0)) {
        b3ChangeDynamicsInfoSetLocalInertiaDiagonal(
        command,
        bodyUniqueId,
        linkIndex,
        args.m_localInertiaDiagonal);
    }
  if (args.m_lateralFriction >= 0) {
    b3ChangeDynamicsInfoSetLateralFriction(command,
    bodyUniqueId,
    linkIndex,
    args.m_lateralFriction);
  }

  if (args.m_spinningFriction >= 0) {
    b3ChangeDynamicsInfoSetSpinningFriction(command,
    bodyUniqueId,
    linkIndex,
    args.m_spinningFriction);
  }

  if (args.m_rollingFriction >= 0) {
    b3ChangeDynamicsInfoSetRollingFriction(command,
    bodyUniqueId,
    linkIndex,
    args.m_rollingFriction);
  }

  if (args.m_linearDamping >= 0) {
    b3ChangeDynamicsInfoSetLinearDamping(command,
    bodyUniqueId,
    args.m_linearDamping);
  }

  if (args.m_angularDamping >= 0) {
    b3ChangeDynamicsInfoSetAngularDamping(command,
    bodyUniqueId,
    args.m_angularDamping);
  }

  if (args.m_restitution >= 0) {
    b3ChangeDynamicsInfoSetRestitution(command,
    bodyUniqueId,
    linkIndex,
    args.m_restitution);
  }

  if (args.m_contactStiffness >= 0 && args.m_contactDamping >= 0) {
    b3ChangeDynamicsInfoSetContactStiffnessAndDamping(command,
    bodyUniqueId,
    linkIndex,
    args.m_contactStiffness,
    args.m_contactDamping);
  }

  if (args.m_frictionAnchor >= 0) {
    b3ChangeDynamicsInfoSetFrictionAnchor(command,
    bodyUniqueId,
    linkIndex,
    args.m_frictionAnchor);
    }
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  return true;
}
