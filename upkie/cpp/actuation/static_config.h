// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include "upkie/cpp/actuation/moteus/PositionResolution.h"
#include "upkie/cpp/actuation/moteus/QueryCommand.h"

namespace upkie::cpp::actuation::static_config {

/*! Resolution settings for all servo commands.
 *
 * \return Resolution settings.
 *
 * For now these settings are common to all interfaces but we can easily turn
 * them into parameters.
 */
inline moteus::PositionResolution position_resolution() noexcept {
  using moteus::Resolution;
  moteus::PositionResolution resolution;
  resolution.position = Resolution::kFloat;
  resolution.velocity = Resolution::kFloat;
  resolution.feedforward_torque = Resolution::kIgnore;
  resolution.kp_scale = Resolution::kFloat;
  resolution.kd_scale = Resolution::kFloat;
  resolution.maximum_torque = Resolution::kFloat;
  resolution.stop_position = Resolution::kIgnore;
  resolution.watchdog_timeout = Resolution::kIgnore;
  return resolution;
}

/*! Query resolution settings for all servo commands.
 *
 * \return Query resolution settings.
 *
 * For now these settings are common to all interfaces but we can easily turn
 * them into parameters.
 */
inline moteus::QueryCommand query_resolution() noexcept {
  using moteus::Resolution;
  moteus::QueryCommand query;
  query.mode = Resolution::kInt16;
  query.position = Resolution::kInt32;
  query.velocity = Resolution::kInt32;
  query.torque = Resolution::kInt32;
  query.q_current = Resolution::kIgnore;
  query.d_current = Resolution::kIgnore;
  query.rezero_state = Resolution::kIgnore;
  query.voltage = Resolution::kInt8;
  query.temperature = Resolution::kInt8;
  query.fault = Resolution::kInt8;
  return query;
}

/*! Get Upkie's servo layout.
 *
 * \return Upkie's servo layout.
 */
inline const ServoLayout servo_layout() noexcept {
  constexpr int kLeftBusID = 1;              // JC1 on the pi3hat
  constexpr int kRightBusID = 3;             // JC3 on the pi3hat
  constexpr double kMaxQdd100Torque = 16.0;  // [N m]
  constexpr double kMaxMj5208Torque = 1.7;   // [N m]
  ServoLayout layout;
  layout.add_servo(1, kLeftBusID, "left_hip", kMaxQdd100Torque);
  layout.add_servo(2, kLeftBusID, "left_knee", kMaxQdd100Torque);
  layout.add_servo(3, kLeftBusID, "left_wheel", kMaxMj5208Torque);
  layout.add_servo(4, kRightBusID, "right_hip", kMaxQdd100Torque);
  layout.add_servo(5, kRightBusID, "right_knee", kMaxQdd100Torque);
  layout.add_servo(6, kRightBusID, "right_wheel", kMaxMj5208Torque);
  return layout;
}

}  // namespace upkie::cpp::actuation::static_config
