// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <string>

//! Exceptions raised by this project.
namespace upkie::cpp::exceptions {

//! Base class for Upkie-related exceptions.
class UpkieError : public std::runtime_error {
 public:
  /*! Create a new error.
   *
   * \param[in] message Error message.
   */
  explicit UpkieError(const std::string& message)
      : std::runtime_error(message), message_(message) {}

  /*! Copy an existing error, adding to the error message.
   *
   * \param[in] other Existing error.
   * \param[in] extra_message Additional error message.
   */
  UpkieError(const UpkieError& other, const std::string& extra_message)
      : std::runtime_error(other.message_ + extra_message),
        message_(other.message_ + extra_message) {}

  //! Error message
  const char* what() const noexcept override { return message_.c_str(); }

 protected:
  //! Complete error message
  std::string message_;
};

}  // namespace upkie::cpp::exceptions
