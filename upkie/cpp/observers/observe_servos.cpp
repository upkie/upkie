// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/cpp/observers/observe_servos.h"

#include <map>
#include <string>
#include <vector>

namespace moteus = upkie::cpp::actuation::moteus;

using palimpsest::Dictionary;

namespace upkie {

void observe_servos(Dictionary& observation,
                    const std::map<int, std::string>& servo_joint_map,
                    const std::vector<moteus::ServoReply>& servo_replies) {
  for (const auto& reply : servo_replies) {
    const int servo_id = reply.id;
    auto it = servo_joint_map.find(servo_id);
    if (it == servo_joint_map.end()) {
      spdlog::error("Unknown servo ID {} in CAN reply", servo_id);
      continue;
    }
    const auto& joint = it->second;

    // The moteus convention is that positive angles correspond to clockwise
    // rotations when looking at the rotor / back of the moteus board. See:
    // https://jpieper.com/2021/04/30/moteus-direction-configuration/
    double position_rev = reply.result.position;
    double velocity_rev_s = reply.result.velocity;
    double position_rad = (2.0 * M_PI) * position_rev;
    double velocity_rad_s = (2.0 * M_PI) * velocity_rev_s;

    auto& servo = observation("servo")(joint);
    servo("d_current") = reply.result.d_current;
    servo("fault") = reply.result.fault;
    servo("mode") = static_cast<unsigned>(reply.result.mode);
    servo("position") = position_rad;
    servo("q_current") = reply.result.q_current;
    servo("temperature") = reply.result.temperature;
    servo("torque") = reply.result.torque;
    servo("velocity") = velocity_rad_s;
    servo("voltage") = reply.result.voltage;
  }
}

}  // namespace upkie
