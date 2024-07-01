// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include "upkie/cpp/actuation/ServoLayout.h"

namespace upkie::cpp::actuation {

/*! Get a sample servo layout.
 *
 * \return Sample servo layout.
 */
inline const ServoLayout get_coffee_machine_layout() noexcept {
  ServoLayout layout;
  layout.add_servo(1, 1, "left_grinder");
  layout.add_servo(3, 1, "left_pump");
  layout.add_servo(5, 2, "right_grinder");
  layout.add_servo(6, 2, "right_pump");
  return layout;
}

}  // namespace upkie::cpp::actuation
