// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <string>

#include "upkie/cpp/exceptions/UpkieError.h"

namespace upkie::cpp::exceptions {

//! Exception raised when an actuation command is incorrect
class PositionCommandError : public UpkieError {
 public:
  /*! Create error.
   *
   * \param[in] message Error message.
   * \param[in] servo_id Servo ID.
   */
  PositionCommandError(const std::string& message, int servo_id)
      : UpkieError(message) {}

  //! Empty destructor
  ~PositionCommandError() throw() {}
};

}  // namespace upkie::cpp::exceptions
