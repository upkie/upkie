// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Inria

#pragma once

#include <string>

#include "upkie/cpp/exceptions/UpkieError.h"

namespace upkie::cpp::exceptions {

//! Exception thrown when a controller fails.
class ControllerError : public UpkieError {
 public:
  /*! Create a controller error.
   *
   * \param[in] name Controller name.
   * \param[in] message Error message.
   */
  ControllerError(const std::string& name, const std::string& message)
      : UpkieError(message), name_(name) {
    std::ostringstream out;
    out << "Failed to update /action/" << name << ": " << message;
    message_ = out.str();
  }

  //! Controller name in
  const std::string& name() const noexcept { return name_; }

 private:
  //! Controller name in observation/
  std::string name_;
};

}  // namespace upkie::cpp::exceptions
