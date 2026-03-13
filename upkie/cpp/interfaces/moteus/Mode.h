// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace upkie::cpp::interfaces {

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

}  // namespace upkie::cpp::interfaces
