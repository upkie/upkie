// SPDX-License-Identifier: Apache-2.0

#include "upkie/cpp/observers/observe_servos.h"

#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "upkie/cpp/exceptions/ServoError.h"

namespace upkie::cpp::observers {

using upkie::cpp::exceptions::ServoError;
using upkie::cpp::interfaces::moteus::ServoReply;

void observe_servos(palimpsest::Dictionary& observation,
                    const std::map<int, std::string>& servo_name_map,
                    const std::vector<ServoReply>& servo_replies) {
  static int strikes = 0;
  static int clean_packets = 0;

  for (const auto& reply : servo_replies) {
    const int servo_id = reply.id;
    auto it = servo_name_map.find(servo_id);
    if (it == servo_name_map.end()) {
      spdlog::error("Unknown servo ID {} in CAN reply", servo_id);
      continue;
    }

    const auto& joint_name = it->second;
    if (std::isnan(reply.result.torque)) {
      ++strikes;
      clean_packets = 0;
      std::ostringstream packet;
      packet << "torque NaN (strike " << strikes << "/3); full packet: {"
             << "mode=" << static_cast<unsigned>(reply.result.mode)
             << ", position=" << reply.result.position
             << ", velocity=" << reply.result.velocity
             << ", torque=" << reply.result.torque
             << ", q_current=" << reply.result.q_current
             << ", d_current=" << reply.result.d_current
             << ", voltage=" << reply.result.voltage
             << ", temperature=" << reply.result.temperature
             << ", fault=" << reply.result.fault
             << ", rezero_state=" << reply.result.rezero_state << "}";
      spdlog::warn("Servo \"{}\" (ID {}): {}", joint_name, servo_id,
                   packet.str());
      if (strikes >= 3) {
        throw ServoError(servo_id, joint_name, packet.str());
      }
      continue;
    }
    if (strikes > 0 && ++clean_packets >= 100) {
      strikes = 0;
      clean_packets = 0;
    }

    // The moteus convention is that positive angles correspond to clockwise
    // rotations when looking at the rotor / back of the moteus board. See:
    // https://jpieper.com/2021/04/30/moteus-direction-configuration/
    double position_rev = reply.result.position;
    double velocity_rev_s = reply.result.velocity;
    double position_rad = (2.0 * M_PI) * position_rev;
    double velocity_rad_s = (2.0 * M_PI) * velocity_rev_s;

    auto& servo = observation("servo")(joint_name);
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

}  // namespace upkie::cpp::observers
