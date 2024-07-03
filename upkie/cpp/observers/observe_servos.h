// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>

#include <map>
#include <string>
#include <vector>

#include "upkie/cpp/actuation/moteus/protocol.h"

namespace upkie::observers {

using upkie::actuation::moteus::ServoReply;

/*! Observe servo measurements.
 *
 * \param[out] observation Dictionary to write observations to.
 * \param[in] servo_joint_map Map from servo ID to joint name.
 * \param[in] servo_replies List of servo replies from the CAN bus.
 */
void observe_servos(palimpsest::Dictionary& observation,
                    const std::map<int, std::string>& servo_joint_map,
                    const std::vector<ServoReply>& servo_replies);

}  // namespace upkie::observers
