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

namespace upkie::actuation {

namespace moteus {

/*! Position command.
 *
 * \note Default values here are also the initial joint values for the mock
 * interface. See the documentation at \ref
 * upkie::MockInterface::ServoState::step.
 */
struct PositionCommand {
  double position = 0.0;
  double velocity = 0.0;
  double feedforward_torque = 0.0;
  double kp_scale = 1.0;
  double kd_scale = 1.0;
  double maximum_torque = 0.0;
  double stop_position = std::numeric_limits<double>::quiet_NaN();
  double watchdog_timeout = 0.0;
};

}  // namespace moteus

}  // namespace upkie::actuation
