// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <string>

#include "upkie/cpp/exceptions/UpkieError.h"

namespace upkie::exceptions {

//! Exception raised when a deserialized object does not have the right type.
class TypeError : public UpkieError {
 public:
  /*! Create a type error.
   *
   * @param[in] file Source file of the instruction that threw the error.
   * @param[in] line Line of code in that file where the throw originates from.
   * @param[in] message Error message.
   */
  TypeError(const std::string& file, unsigned line, const std::string& message)
      : UpkieError(file, line, message) {}

  /*! Copy an existing error, adding to the error message.
   *
   * @param[in] other Existing error.
   * @param[in] extra_message Additional error message.
   */
  TypeError(const TypeError& other, const std::string& extra_message)
      : UpkieError(other, extra_message) {}

  //! Empty destructor
  ~TypeError() throw() {}
};

}  // namespace upkie::exceptions
