// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#ifndef ROBOT_SIMULATOR_CLIENT_API_H
#define ROBOT_SIMULATOR_CLIENT_API_H

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "SharedMemory/PhysicsClientC_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "Bullet3Common/b3Logging.h"
#include <Eigen/Dense>


struct RobotSimulatorChangeDynamicsArgs{
    double m_mass;
    double m_lateralFriction;
    double m_spinningFriction;
    double m_rollingFriction;
    double m_restitution;
    double m_linearDamping;
    double m_angularDamping;
    double m_contactStiffness;
    double m_contactDamping;
    int m_frictionAnchor;
    int m_activationState;
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
          m_localInertiaDiagonal{0, 0, 0}
        {
        }
};
class RobotSimulatorClientAPI : public b3RobotSimulatorClientAPI {
 public:
    bool changeDynamics(int bodyUniqueId,
      int linkIndex,
      RobotSimulatorChangeDynamicsArgs& args);
};


#endif  // ROBOT_SIMULATOR_CLIENT_API_H
