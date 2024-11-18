// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2024 Inria

#pragma once

#include <string>

#include "upkie/cpp/exceptions/UpkieError.h"

namespace upkie::cpp::exceptions {

//! Exception for filter errors
class ServoError : public UpkieError {
 public:
  /*! Exception constructor.
   *
   * \param[in] servo_id Servo ID.
   * \param[in] joint_name Name of the joint actuated by the servo.
   * \param[in] message Error message.
   */
  ServoError(const int servo_id, const std::string& joint_name,
             const std::string& message)
      : UpkieError(message) {
    std::ostringstream out;
    out << "Servo \"" << joint_name << "\" (ID " << servo_id
        << ") error: " << message;
    message_ = out.str();
  }
};

}  // namespace upkie::cpp::exceptions
