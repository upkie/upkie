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

#include "upkie/cpp/interfaces/moteus/Resolution.h"

namespace upkie::cpp::interfaces {

namespace moteus {

//! Resolution settings for query commands.
struct QueryCommand {
  //! Resolution for control mode values.
  Resolution mode = Resolution::kInt16;

  //! Resolution for position values.
  Resolution position = Resolution::kInt16;

  //! Resolution for velocity values.
  Resolution velocity = Resolution::kInt16;

  //! Resolution for torque values.
  Resolution torque = Resolution::kInt16;

  //! Resolution for q-axis current values.
  Resolution q_current = Resolution::kIgnore;

  //! Resolution for d-axis current values.
  Resolution d_current = Resolution::kIgnore;

  //! Resolution for rezero state values.
  Resolution rezero_state = Resolution::kIgnore;

  //! Resolution for voltage values.
  Resolution voltage = Resolution::kInt8;

  //! Resolution for temperature values.
  Resolution temperature = Resolution::kInt8;

  //! Resolution for fault values.
  Resolution fault = Resolution::kInt8;

  //! Check if any resolution is set to a non-ignore value.
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

}  // namespace upkie::cpp::interfaces
