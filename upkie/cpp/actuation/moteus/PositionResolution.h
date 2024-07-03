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

namespace upkie::actuation {

namespace moteus {

struct PositionResolution {
  Resolution position = Resolution::kFloat;
  Resolution velocity = Resolution::kFloat;
  Resolution feedforward_torque = Resolution::kFloat;
  Resolution kp_scale = Resolution::kFloat;
  Resolution kd_scale = Resolution::kFloat;
  Resolution maximum_torque = Resolution::kIgnore;
  Resolution stop_position = Resolution::kFloat;
  Resolution watchdog_timeout = Resolution::kFloat;
};

}  // namespace moteus

}  // namespace upkie::actuation
