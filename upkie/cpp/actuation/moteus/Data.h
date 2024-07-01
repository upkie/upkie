// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *     Copyright 2020 Josh Pieper, jjp@pobox.com.
 *     SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "upkie/cpp/actuation/moteus/ServoCommand.h"
#include "upkie/cpp/actuation/moteus/ServoReply.h"
#include "upkie/cpp/actuation/moteus/Span.h"

namespace upkie {

namespace moteus {

/*! Data for one control cycle.
 *
 * This describes what you would like to do in a given control cycle in terms
 * of sending commands or querying data.
 */
struct Data {
  //! Pointers to servo command buffers.
  Span<ServoCommand> commands;

  //! Pointers to servo reply buffers.
  Span<ServoReply> replies;
};

}  // namespace moteus

}  // namespace upkie
