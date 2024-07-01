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

namespace upkie {

namespace moteus {

//! Control mode
enum class Mode {
  kStopped = 0,
  kFault = 1,
  kEnabling = 2,
  kCalibrating = 3,
  kCalibrationComplete = 4,
  kPwm = 5,
  kVoltage = 6,
  kVoltageFoc = 7,
  kVoltageDq = 8,
  kCurrent = 9,
  kPosition = 10,
  kPositionTimeout = 11,
  kZeroVelocity = 12,
  kNumModes,
};

}  // namespace moteus

}  // namespace upkie
