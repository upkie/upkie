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

#include <limits>

#include "upkie/cpp/interfaces/moteus/Mode.h"

namespace upkie::cpp::interfaces {

namespace moteus {

//! Result data from moteus query commands.
struct QueryResult {
  //! Control mode
  Mode mode = Mode::kStopped;

  //! Angular position in [rev]
  double position = std::numeric_limits<double>::quiet_NaN();

  //! Angular velocity in [rev] / [s]
  double velocity = std::numeric_limits<double>::quiet_NaN();

  //! Torque in [N m]
  double torque = std::numeric_limits<double>::quiet_NaN();

  //! Q-phase current in [A]
  double q_current = std::numeric_limits<double>::quiet_NaN();

  //! D-phase current in [A]
  double d_current = std::numeric_limits<double>::quiet_NaN();

  /*! Boolean flag used to figure out the output zero of a servo.
   *
   * When this flag is true, we assume the output is currently in the same
   * sextant (for a qdd100) as its zero. We then recover absolute calibration
   * based on the absolute encoder on the motor side.
   */
  bool rezero_state = false;

  //! Supply voltage in [V].
  double voltage = std::numeric_limits<double>::quiet_NaN();

  //! Controller temperature in [°C].
  double temperature = std::numeric_limits<double>::quiet_NaN();

  //! Fault code (0 = no fault).
  int fault = 0;
};

}  // namespace moteus

}  // namespace upkie::cpp::interfaces
