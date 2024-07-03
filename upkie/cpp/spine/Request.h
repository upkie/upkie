// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <cstdint>

namespace upkie::spine {

enum class Request : uint32_t {
  kNone = 0,  // no active request
  kObservation = 1,
  kAction = 2,
  kStart = 3,
  kStop = 4,
  kError = 5  // last request was invalid
};

}  // namespace upkie::spine
