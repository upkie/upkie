// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>

#include "upkie/cpp/exceptions/UpkieError.h"

namespace upkie::cpp::exceptions {

//! Exception for filter errors
class FilterError : public UpkieError {
 public:
  /*! Constructor.
   *
   * \param[in] message Error message.
   */
  explicit FilterError(const std::string& message) : UpkieError(message) {}
};

}  // namespace upkie::cpp::exceptions
