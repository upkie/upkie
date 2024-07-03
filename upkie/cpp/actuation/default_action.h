// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 Inria

#pragma once

namespace upkie::actuation {

namespace default_action {

constexpr double kFeedforwardTorque = 0.0;  // N.m
constexpr double kVelocity = 0.0;           // rad/s
constexpr double kKpScale = 1.0;            // no unit
constexpr double kKdScale = 1.0;            // no unit
constexpr double kMaximumTorque = 1.0;      // N.m

}  // namespace default_action

}  // namespace upkie::actuation
