// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <string>

namespace upkie::cpp::exceptions {

//! Exception for filter errors
class FilterError : public std::runtime_error {
 public:
  /*! Constructor.
   *
   * \param[in] message Error message.
   */
  explicit FilterError(const std::string& message) throw()
      : std::runtime_error(message.c_str()) {}

  /*! Get error message.
   *
   * \return Error message.
   */
  const char* what() const throw() { return std::runtime_error::what(); }
};

}  // namespace upkie::cpp::exceptions
