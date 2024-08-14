// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2024 Inria

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
  explicit FilterError(const std::string& message) throw()
      : UpkieError(message) {}
};

}  // namespace upkie::cpp::exceptions
