// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <string>

#include "LinearMath/btVector3.h"
#include "upkie/cpp/actuation/ImuData.h"

namespace upkie::cpp::actuation::bullet {

//! Parameters for a call to Bullet's applyExternalForce
struct ExternalForce {
  //! Integer specifying if the force is in the link or world frame.
  int flags;

  //! Force to apply, in [N].
  btVector3 force;
};

}  // namespace upkie::cpp::actuation::bullet
