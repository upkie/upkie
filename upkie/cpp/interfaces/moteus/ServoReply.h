// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "upkie/cpp/interfaces/moteus/QueryResult.h"

namespace upkie::cpp::interfaces {

namespace moteus {

//! Reply from servo
struct ServoReply {
  //! Servo identifier
  int id = 0;

  //! Content of servo reply, with resolution from \ref ServoCommand::query
  QueryResult result;
};

}  // namespace moteus

}  // namespace upkie::cpp::interfaces
