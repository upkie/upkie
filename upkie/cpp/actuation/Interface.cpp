// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2023 Inria
/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *     SPDX-License-Identifier: Apache-2.0
 *     Copyright 2020 Josh Pieper, jjp@pobox.com.
 */

#include "upkie/cpp/actuation/Interface.h"

#include <palimpsest/Dictionary.h>

#include <limits>
#include <map>
#include <string>
#include <vector>

#include "upkie/cpp/actuation/default_action.h"
#include "upkie/cpp/actuation/moteus/Mode.h"
#include "upkie/cpp/actuation/moteus/ServoCommand.h"
#include "upkie/cpp/exceptions/PositionCommandError.h"
#include "upkie/cpp/model/joints.h"

namespace upkie::cpp::actuation {

//! Temporary constant for maximum-torque checks
constexpr double kLeastReasonableMaximumTorque = 16.0;  // [N m]

using exceptions::PositionCommandError;

Interface::Interface() : servo_layout_(static_config::servo_layout()) {
  for (const auto& id_joint : servo_layout_.servo_joint_map()) {
    const int joint_id = id_joint.first;
    const std::string& joint_name = id_joint.second.name;
    servo_name_map_.insert({joint_id, joint_name});
    spdlog::info("Inserted {}, {}", joint_id, joint_name);
  }

  auto query_resolution = static_config::query_resolution();
  auto position_resolution = static_config::position_resolution();
  for (const auto& pair : servo_layout_.servo_bus_map()) {
    commands_.push_back({});
    commands_.back().id = pair.first;
    commands_.back().resolution = position_resolution;
    commands_.back().query = query_resolution;
  }

  replies_.resize(commands_.size());
  data_.commands = {commands_.data(), commands_.size()};
  data_.replies = {replies_.data(), replies_.size()};
}

void Interface::reset_action(Dictionary& action) {
  for (const auto& id_joint : servo_layout_.servo_joint_map()) {
    const std::string& joint_name = id_joint.second.name;
    auto& servo_action = action("servo")(joint_name);
    servo_action("feedforward_torque") = default_action::kFeedforwardTorque;
    servo_action("position") = std::numeric_limits<double>::quiet_NaN();
    servo_action("velocity") = default_action::kVelocity;
    servo_action("kp_scale") = default_action::kKpScale;
    servo_action("kd_scale") = default_action::kKdScale;
    servo_action("maximum_torque") = default_action::kMaximumTorque;
  }
}

void Interface::write_position_commands(const Dictionary& action) {
  if (!action.has("servo")) {
    spdlog::warn("No position command at key \"servo\" of action");
    return;
  }

  const auto& servo = action("servo");
  for (auto& command : commands_) {
    const int servo_id = command.id;
    auto it = servo_name_map_.find(servo_id);
    if (it == servo_name_map_.end()) {
      spdlog::error("Unknown servo ID {} in CAN command", servo_id);
      throw PositionCommandError("Unknown servo ID", servo_id);
    }
    const std::string& joint_name = it->second;
    if (!servo.has(joint_name)) {
      spdlog::error("No action for joint {}", joint_name);
      throw PositionCommandError("No action", servo_id);
    }
    const auto& servo_action = servo(joint_name);
    if (!servo_action.has("position")) {
      spdlog::error("No position command for joint {}", joint_name);
      throw PositionCommandError("No position command", servo_id);
    }

    const double feedforward_torque = servo_action.get<double>(
        "feedforward_torque", default_action::kFeedforwardTorque);
    const double position_rad = servo_action("position");
    const double velocity_rad_s =
        servo_action.get<double>("velocity", default_action::kVelocity);
    const double kp_scale =
        servo_action.get<double>("kp_scale", default_action::kKpScale);
    const double kd_scale =
        servo_action.get<double>("kd_scale", default_action::kKdScale);
    const double maximum_torque = servo_action.get<double>(
        "maximum_torque", default_action::kMaximumTorque);
    if (maximum_torque < 0.0 ||
        maximum_torque > kLeastReasonableMaximumTorque) {
      spdlog::error("Unreasonable maximum torque ({} N m) for joint {}",
                    maximum_torque, joint_name);
      throw PositionCommandError("Invalid maximum torque", servo_id);
    }

    // The moteus convention is that positive angles correspond to clockwise
    // rotations when looking at the rotor / back of the moteus board. See:
    // https://jpieper.com/2021/04/30/moteus-direction-configuration/
    const double position_rev = position_rad / (2.0 * M_PI);
    const double velocity_rev_s = velocity_rad_s / (2.0 * M_PI);

    command.mode = moteus::Mode::kPosition;
    command.position.feedforward_torque = feedforward_torque;
    command.position.position = position_rev;
    command.position.velocity = velocity_rev_s;
    command.position.kp_scale = kp_scale;
    command.position.kd_scale = kd_scale;
    command.position.maximum_torque = maximum_torque;
  }
}

}  // namespace upkie::cpp::actuation
