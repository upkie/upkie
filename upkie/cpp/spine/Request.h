// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <cstdint>

//! Main control loop between agents and actuators.
namespace upkie::spine {

//! Request flag used for shared-memory inter-process communication.
enum class Request : uint32_t {
  //! Flag set when there is no active request.
  kNone = 0,

  //! Flag set to indicate an observation is requested.
  kObservation = 1,

  //! Flag set to indicate an action has been supplied.
  kAction = 2,

  //! Flag set to start the spine.
  kStart = 3,

  //! Flag set to stop the spine.
  kStop = 4,

  //! Flag set when the last request was invalid.
  kError = 5
};

}  // namespace upkie::spine
