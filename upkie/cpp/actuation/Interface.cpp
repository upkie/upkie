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

namespace upkie::cpp::actuation {

void Interface::initialize_action(Dictionary& action) {
  for (const auto& id_joint : servo_layout_.servo_joint_map()) {
    const std::string& joint_name = id_joint.second;
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
  using Mode = moteus::Mode;
  if (!action.has("servo")) {
    spdlog::warn("No position command at key \"servo\" of action");
    return;
  }

  const auto& servo = action("servo");
  const auto& servo_joint_map = servo_layout_.servo_joint_map();
  for (auto& command : commands_) {
    const int servo_id = command.id;
    auto it = servo_joint_map.find(servo_id);
    if (it == servo_joint_map.end()) {
      spdlog::error("Unknown servo ID {} in CAN command", servo_id);
      command.mode = Mode::kStopped;
      continue;
    }
    const auto& joint = it->second;
    if (!servo.has(joint)) {
      spdlog::error("No action for joint {} (servo ID={})", joint, servo_id);
      command.mode = Mode::kStopped;
      continue;
    }
    const auto& servo_action = servo(joint);
    if (!servo_action.has("position")) {
      spdlog::error("No position command for joint {} (servo ID={})", joint,
                    servo_id);
      command.mode = Mode::kStopped;
      continue;
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

    // The moteus convention is that positive angles correspond to clockwise
    // rotations when looking at the rotor / back of the moteus board. See:
    // https://jpieper.com/2021/04/30/moteus-direction-configuration/
    const double position_rev = position_rad / (2.0 * M_PI);
    const double velocity_rev_s = velocity_rad_s / (2.0 * M_PI);

    command.mode = Mode::kPosition;
    command.position.feedforward_torque = feedforward_torque;
    command.position.position = position_rev;
    command.position.velocity = velocity_rev_s;
    command.position.kp_scale = kp_scale;
    command.position.kd_scale = kd_scale;
    command.position.maximum_torque = maximum_torque;
  }
}

}  // namespace upkie::cpp::actuation
