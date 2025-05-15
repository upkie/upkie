// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Inria

#include "upkie/cpp/controllers/WheelBalancer.h"

namespace upkie::cpp::controllers {

WheelBalancer::WheelBalancer(const Parameters& params)
    : params_(params), ground_velocity_(0.0) {}

void WheelBalancer::reset(const Dictionary& config) {
  params_.configure(config);
  ground_velocity_ = 0.0;
}

void WheelBalancer::read(const Dictionary& observation,
                         const Dictionary& action) {
  double pitch = observation("base_orientation")("pitch");
  if (std::abs(pitch) > params_.fall_pitch) {
    ground_velocity_ = 0.0;
    return;
  }

  // double ground_position = observation("wheel_odometry")("position");
  // bool floor_contact = observation("floor_contact")("contact");
  double target_pitch = 0.0;  // [rad]
  double pitch_error = target_pitch - pitch;
  double kp = 1.0;
  ground_velocity_ = kp * pitch_error;
}

void WheelBalancer::write(Dictionary& action) {
  const double ref_wheel_velocity = ground_velocity_ / params_.wheel_radius;
  double& left_wheel_velocity = action("servo")("left_wheel")("velocity");
  double& right_wheel_velocity = action("servo")("right_wheel")("velocity");
  left_wheel_velocity += ref_wheel_velocity;
  right_wheel_velocity -= ref_wheel_velocity;
}

}  // namespace upkie::cpp::controllers
