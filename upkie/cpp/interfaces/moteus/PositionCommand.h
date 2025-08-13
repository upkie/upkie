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

namespace upkie::cpp::interfaces {

namespace moteus {

/*! Position command.
 *
 * \note Default values here are also the initial joint values for the mock
 * interface.
 */
struct PositionCommand {
  //! Target position in revolutions.
  double position = 0.0;

  //! Target velocity in rev/s.
  double velocity = 0.0;

  //! Feedforward torque in N⋅m.
  double feedforward_torque = 0.0;

  //! Proportional gain scaling factor.
  double kp_scale = 1.0;

  //! Derivative gain scaling factor.
  double kd_scale = 1.0;

  //! Maximum torque limit in N⋅m.
  double maximum_torque = 0.0;

  //! Stop position in revolutions.
  double stop_position = std::numeric_limits<double>::quiet_NaN();

  //! Watchdog timeout in seconds.
  double watchdog_timeout = 0.0;
};

}  // namespace moteus

}  // namespace upkie::cpp::interfaces
