// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 Inria

#pragma once

namespace upkie::cpp::actuation {

//! Default values for servo actions.
namespace default_action {

//! Feedforward torque in N.m
constexpr double kFeedforwardTorque = 0.0;

//! Joint velocity in rad/s.
constexpr double kVelocity = 0.0;

//! Scaling coefficient (no unit) applied to position feedback.
constexpr double kKpScale = 1.0;

//! Scaling coefficient (no unit) applied to velocity feedback.
constexpr double kKdScale = 1.0;

//! Maximum torque the servo is allowed to apply, in N.m.
constexpr double kMaximumTorque = 1.0;

}  // namespace default_action

}  // namespace upkie::cpp::actuation
