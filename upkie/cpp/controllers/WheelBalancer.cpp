// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Inria

#include "upkie/cpp/controllers/WheelBalancer.h"

#include <algorithm>

#include "upkie/cpp/utils/low_pass_filter.h"

namespace upkie::cpp::controllers {

constexpr double kAirReturnPeriod = 1.0;       // s
constexpr double kMaxIntegralVelocity = 10.0;  // m/s
constexpr double kMaxTargetDistance = 1.0;     // m
constexpr double kGainScale = 2.0;
constexpr double kTurningGainScale = 2.0;

using upkie::cpp::utils::low_pass_filter;

WheelBalancer::WheelBalancer(const Parameters& params)
    : params_(params),
      ground_velocity_(0.0),
      integral_velocity_(0.0),
      target_ground_position_(0.0),
      target_yaw_velocity_(0.0) {}

void WheelBalancer::reset(const Dictionary& config) {
  params_.configure(config);

  ground_velocity_ = 0.0;
  integral_velocity_ = 0.0;
  target_ground_position_ = 0.0;
  target_yaw_velocity_ = 0.0;
}

void WheelBalancer::read(const Dictionary& observation,
                         const Dictionary& action) {
  double target_ground_velocity = 0.0;
  if (action.has("bullet")) {
    target_ground_velocity =
        action("bullet").get("target_ground_velocity", 0.0);
    target_yaw_velocity_ = action("bullet").get("target_yaw_velocity", 0.0);
  }

  const double dt = params_.dt;
  const double pitch = observation("base_orientation")("pitch");
  if (std::abs(pitch) > params_.fall_pitch) {
    ground_velocity_ = 0.0;
    return;
  }

  const double ground_position = observation("wheel_odometry")("position");
  const bool floor_contact = observation("floor_contact")("contact");
  double target_pitch = 0.0;  // rad
  Eigen::Vector2d error = {
      target_ground_position_ - ground_position,
      target_pitch - pitch,
  };

  Eigen::Vector2d kp = {params_.position_damping, params_.pitch_damping};
  Eigen::Vector2d ki = {params_.position_stiffness, params_.pitch_stiffness};

  if (floor_contact) {
    integral_velocity_ += ki.dot(error) * dt;
    integral_velocity_ = std::clamp(integral_velocity_, -kMaxIntegralVelocity,
                                    +kMaxIntegralVelocity);
    target_ground_position_ += target_ground_velocity * dt;
    target_ground_position_ = std::clamp(target_ground_position_,
                                         ground_position - kMaxTargetDistance,
                                         ground_position + kMaxTargetDistance);
  } else /* no contact */ {
    integral_velocity_ =
        low_pass_filter(integral_velocity_, kAirReturnPeriod, 0.0, dt);
    // We soft-reset the target ground velocity: either takeoff detection is a
    // false positive and we should resume close to the pre-takeoff state, or
    // the robot is really in the air and the user should stop smashing the
    // joystick like a bittern ;p
    target_ground_position_ = low_pass_filter(
        target_ground_position_, kAirReturnPeriod, ground_position, dt);
  }

  // Non-minimum phase trick, see class documentation for details
  double trick_velocity = -target_ground_velocity;
  ground_velocity_ = trick_velocity - kp.dot(error) - integral_velocity_;
  ground_velocity_ = std::clamp(ground_velocity_, -params_.max_ground_velocity,
                                +params_.max_ground_velocity);
}

void WheelBalancer::write(Dictionary& action) {
  const double wheel_radius = params_.wheel_radius;
  const double ref_wheel_velocity = ground_velocity_ / wheel_radius;
  double& left_wheel_velocity = action("servo")("left_wheel")("velocity");
  double& right_wheel_velocity = action("servo")("right_wheel")("velocity");
  left_wheel_velocity += ref_wheel_velocity;
  right_wheel_velocity -= ref_wheel_velocity;

  const double yaw_to_wheel = params_.contact_radius / wheel_radius;
  left_wheel_velocity += yaw_to_wheel * target_yaw_velocity_;
  right_wheel_velocity += yaw_to_wheel * target_yaw_velocity_;

  const double turning_prob =
      (std::abs(target_yaw_velocity_) > params_.stiff_yaw_velocity) ? 1.0 : 0.0;
  const double kp_scale = kGainScale + kTurningGainScale * turning_prob;
  const double kd_scale = kGainScale + kTurningGainScale * turning_prob;
  for (const auto& name :
       {"left_hip", "left_knee", "right_hip", "right_knee"}) {
    auto& servo_action = action("servo")(name);
    servo_action("kp_scale") = kp_scale;
    servo_action("kd_scale") = kd_scale;
  }
}

}  // namespace upkie::cpp::controllers
