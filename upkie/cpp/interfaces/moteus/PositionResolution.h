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

//! Resolution settings for position control commands.
struct PositionResolution {
  //! Resolution for position values.
  Resolution position = Resolution::kFloat;

  //! Resolution for velocity values.
  Resolution velocity = Resolution::kFloat;

  //! Resolution for feedforward torque values.
  Resolution feedforward_torque = Resolution::kFloat;

  //! Resolution for proportional gain scaling values.
  Resolution kp_scale = Resolution::kFloat;

  //! Resolution for derivative gain scaling values.
  Resolution kd_scale = Resolution::kFloat;

  //! Resolution for maximum torque values.
  Resolution maximum_torque = Resolution::kIgnore;

  //! Resolution for stop position values.
  Resolution stop_position = Resolution::kFloat;

  //! Resolution for watchdog timeout values.
  Resolution watchdog_timeout = Resolution::kFloat;
};

}  // namespace moteus

}  // namespace upkie::cpp::interfaces
