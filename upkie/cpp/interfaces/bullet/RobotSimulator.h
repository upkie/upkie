// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <Eigen/Dense>

#include "Bullet3Common/b3Logging.h"
#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "SharedMemory/PhysicsClientC_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "upkie/cpp/actuation/bullet/RobotSimulatorChangeDynamicsArgs.h"

namespace upkie::cpp::actuation::bullet {

//! Child class to enable modification of inertia matrices.
class RobotSimulatorClientAPI : public b3RobotSimulatorClientAPI {
 public:
  /*! Modifies the dynamic properties of a link.
   *
   * This is a variant of b3RobotSimulatorClientAPI::changeDynamics that allows
   * updating inertia matrices as well. Its usage is identical to the original.
   *
   * \param[in] bodyUniqueId The body which links will be modified
   * \param[in] linkIndex The link id
   * \param[in] args Structure which contains the values to modify
   * \return True if changes were applied successfully, false if there was an
   *     issue.
   */
  bool changeDynamics(int bodyUniqueId, int linkIndex,
                      RobotSimulatorChangeDynamicsArgs& args);
};

}  // namespace upkie::cpp::actuation::bullet
