// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <Eigen/Dense>

#include "Bullet3Common/b3Logging.h"
#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "SharedMemory/PhysicsClientC_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"

namespace upkie::cpp::actuation::bullet {

//! Child class to enable modification of inertia matrices.
class RobotSimulatorClientAPI : public b3RobotSimulatorClientAPI {
 public:
  /*! Modifies the properties of a link
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

}  // namespace upkie::cpp::actuation::bullet
