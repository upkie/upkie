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

#include "upkie/observers/WheelOdometry.h"

namespace upkie::observers {

WheelOdometry::WheelOdometry(const Parameters& params)
    : params_(params), position_(0.0), velocity_(0.0) {}

void WheelOdometry::reset(const Dictionary& config) {
  params_.configure(config);
  position_ = 0.0;
  velocity_ = 0.0;
}

void WheelOdometry::read(const Dictionary& observation) {
  const auto& floor_contact = observation("floor_contact");
  if (!floor_contact("contact")) {
    return;
  }
  const auto& servo = observation("servo");
  velocity_ = compute_average_velocity(floor_contact, servo);
  position_ += velocity_ * params_.dt;
}

double WheelOdometry::compute_average_velocity(const Dictionary& floor_contact,
                                               const Dictionary& servo) {
  if (params_.signed_radius.empty()) {
    throw std::runtime_error(
        "[WheelOdometry] Observer not configured: 'signed_radius' is empty");
  }

  double velocity_sum = 0.0;
  unsigned nb_wheels_in_contact = 0u;
  for (const auto& wheel_radius_pair : params_.signed_radius) {
    const auto& wheel = wheel_radius_pair.first;
    const bool wheel_contact = floor_contact(wheel).get<bool>("contact");
    if (!wheel_contact) {
      continue;
    }
    const double wheel_velocity = servo(wheel)("velocity");  // [rad] / [s]
    const double signed_radius = wheel_radius_pair.second;
    const double linear_velocity = signed_radius * wheel_velocity;  // [m] / [s]
    velocity_sum += linear_velocity;
    ++nb_wheels_in_contact;
  }

  if (nb_wheels_in_contact == 0u) {
    spdlog::warn("[WheelOdometry] Contact detected, but no wheel in contact?");
    return 0.0;
  }

  return (velocity_sum / nb_wheels_in_contact);
}

void WheelOdometry::write(Dictionary& observation) {
  auto& output = observation(prefix());
  output("position") = position_;
  output("velocity") = velocity_;
}

}  // namespace upkie::observers
