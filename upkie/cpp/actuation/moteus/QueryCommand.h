// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *     Copyright 2020 Josh Pieper, jjp@pobox.com.
 *     SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "upkie/cpp/actuation/moteus/Resolution.h"

namespace upkie {

namespace moteus {

struct QueryCommand {
  Resolution mode = Resolution::kInt16;
  Resolution position = Resolution::kInt16;
  Resolution velocity = Resolution::kInt16;
  Resolution torque = Resolution::kInt16;
  Resolution q_current = Resolution::kIgnore;
  Resolution d_current = Resolution::kIgnore;
  Resolution rezero_state = Resolution::kIgnore;
  Resolution voltage = Resolution::kInt8;
  Resolution temperature = Resolution::kInt8;
  Resolution fault = Resolution::kInt8;

  bool any_set() const {
    return mode != Resolution::kIgnore || position != Resolution::kIgnore ||
           velocity != Resolution::kIgnore || torque != Resolution::kIgnore ||
           q_current != Resolution::kIgnore ||
           d_current != Resolution::kIgnore ||
           rezero_state != Resolution::kIgnore ||
           voltage != Resolution::kIgnore ||
           temperature != Resolution::kIgnore || fault != Resolution::kIgnore;
  }
};

}  // namespace moteus

}  // namespace upkie
