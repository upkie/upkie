/*
 * Copyright 2022 Stéphane Caron
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "upkie/observers/WheelContact.h"

#include "vulp/utils/low_pass_filter.h"

namespace upkie::observers {

using vulp::utils::low_pass_filter;

WheelContact::WheelContact(const Parameters& params)
    : params_(params),
      abs_acceleration_(0.0),
      abs_torque_(0.0),
      contact_(false),
      inertia_(0.0),
      velocity_(0.0) {}

void WheelContact::observe(const double torque, const double velocity,
                           const double dt) noexcept {
  if (params_.cutoff_period < 1e-6) {
    spdlog::warn("[WheelContact] Observer is not configured");
    return;
  }

  const double prev_velocity = velocity_;
  velocity_ = low_pass_filter(velocity_, params_.cutoff_period, velocity, dt);
  const double new_acceleration = (velocity_ - prev_velocity) / dt;
  abs_acceleration_ = low_pass_filter(abs_acceleration_, params_.cutoff_period,
                                      std::abs(new_acceleration), dt);
  abs_torque_ =
      low_pass_filter(abs_torque_, params_.cutoff_period, std::abs(torque), dt);

  // Don't detect touchdown when averaged absolute acceleration or torque are
  // too low. Typically when the robot is in the air torques are low. This
  // avoids false positives when the air wheel acceleration happens to be low.
  if (!contact_ && (abs_acceleration_ < params_.min_touchdown_acceleration ||
                    abs_torque_ < params_.min_touchdown_torque)) {
    return;
  }

  inertia_ = abs_torque_ / (abs_acceleration_ + 1e-4);
  if (inertia_ < params_.liftoff_inertia) {
    contact_ = false;
  } else if (inertia_ > params_.touchdown_inertia) {
    contact_ = true;
  }
}

}  // namespace upkie::observers
