// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <cstdint>

//! Main control loop between agents and actuators.
namespace upkie::spine {

//! Request flag used for shared-memory inter-process communication.
enum class Request : uint32_t {
  kNone = 0,  // no active request
  kObservation = 1,
  kAction = 2,
  kStart = 3,
  kStop = 4,
  kError = 5  // last request was invalid
};

}  // namespace upkie::spine
