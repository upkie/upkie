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

#include "upkie/observers/FloorContact.h"

#include <palimpsest/exceptions/KeyError.h>

#include "vulp/utils/low_pass_filter.h"

namespace upkie::observers {

using palimpsest::exceptions::KeyError;
using vulp::utils::low_pass_filter;

FloorContact::FloorContact(const Parameters& params)
    : params_(params), upper_leg_torque_(0.0), contact_(false) {
  for (const auto& wheel : params_.wheels) {
    wheel_contacts_.insert(
        {{wheel, WheelContact(params_.wheel_contact_params)}});
  }
}

void FloorContact::reset(const Dictionary& config) {
  params_.configure(config);
  upper_leg_torque_ = 0.0;
  contact_ = false;
  wheel_contacts_.clear();
  for (const auto& wheel : params_.wheels) {
    wheel_contacts_.insert(
        {{wheel, WheelContact(params_.wheel_contact_params)}});
  }
}

void FloorContact::read(const Dictionary& observation) {
  if (params_.incomplete()) {
    spdlog::warn("[FloorContact] Observer is not configured");
    return;
  }
  if (!observation.has("servo")) {
    return;
  }
  const bool at_least_one_wheel_contact = check_wheel_contacts(observation);
  update_upper_leg_torque(observation);
  contact_ = (at_least_one_wheel_contact ||
              (upper_leg_torque_ > params_.upper_leg_torque_threshold));
}

bool FloorContact::check_wheel_contacts(const Dictionary& observation) {
  const auto& servo = observation("servo");
  bool at_least_one_contact = false;
  for (const auto& wheel : params_.wheels) {
    auto it = wheel_contacts_.find(wheel);  // found by construction
    auto& wheel_contact = it->second;
    try {
      double velocity = servo(wheel)("velocity");
      double torque = servo(wheel)("torque");
      wheel_contact.observe(torque, velocity, params_.dt);
      if (observation.has("joystick") &&
          observation("joystick").get<bool>("cross_button")) {
        wheel_contact.reset_contact();
      } else if (wheel_contact.contact()) {
        at_least_one_contact = true;
      }
    } catch (const KeyError& e) {
      spdlog::warn("[FloorContact] No observation for {} yet", wheel);
      continue;
    }
  }
  return at_least_one_contact;
}

void FloorContact::update_upper_leg_torque(const Dictionary& observation) {
  const auto& servo = observation("servo");
  double squared_torques = 0.0;
  for (const auto& joint : params_.upper_leg_joints) {
    try {
      const double torque = servo(joint)("torque");
      squared_torques += torque * torque;
    } catch (const KeyError& e) {
      spdlog::warn("[FloorContact] No observation for {} yet", joint);
      continue;
    }
  }
  constexpr double kTorqueCutoffPeriod = 0.01;  // [s]
  const double new_torque = std::sqrt(squared_torques);
  upper_leg_torque_ = low_pass_filter(upper_leg_torque_, kTorqueCutoffPeriod,
                                      new_torque, params_.dt);
}

void FloorContact::write(Dictionary& observation) {
  auto& output = observation(prefix());
  output("contact") = contact_;
  output("upper_leg_torque") = upper_leg_torque_;
  for (const auto& wheel : params_.wheels) {
    const auto it = wheel_contacts_.find(wheel);  // found by construction
    const auto& wheel_contact = it->second;
    output(wheel)("abs_acceleration") = wheel_contact.abs_acceleration();
    output(wheel)("abs_torque") = wheel_contact.abs_torque();
    output(wheel)("contact") = wheel_contact.contact();
    output(wheel)("inertia") = wheel_contact.inertia();
  }
}

}  // namespace upkie::observers
