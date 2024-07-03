// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <string>
#include <vector>

#include "upkie/cpp/actuation/ServoLayout.h"

namespace upkie::model {

using upkie::actuation::ServoLayout;

constexpr int kLeftBusID = 1;   // JC1
constexpr int kRightBusID = 3;  // JC3

/*! Get Upkie's servo layout.
 *
 * \return Upkie's servo layout.
 */
inline const ServoLayout servo_layout() noexcept {
  ServoLayout layout;
  layout.add_servo(1, kLeftBusID, "left_hip");
  layout.add_servo(2, kLeftBusID, "left_knee");
  layout.add_servo(3, kLeftBusID, "left_wheel");
  layout.add_servo(4, kRightBusID, "right_hip");
  layout.add_servo(5, kRightBusID, "right_knee");
  layout.add_servo(6, kRightBusID, "right_wheel");
  return layout;
}

}  // namespace upkie::model
