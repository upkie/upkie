// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
// Copyright 2024 Inria

#pragma once

#include <string>

#include "upkie/cpp/exceptions/UpkieError.h"

namespace upkie::cpp::exceptions {

//! Exception thrown when an observer fails.
class ObserverError : public UpkieError {
 public:
  /*! Create an observer error.
   *
   * \param[in] prefix Observer prefix.
   * \param[in] key Key that was not found.
   */
  ObserverError(const std::string& prefix, const std::string& key)
      : UpkieError(key), prefix_(prefix), key_(key) {
    std::ostringstream out;
    out << "Failed to update /observation/" << prefix << ": key \"" << key
        << "\" not found";
    message_ = out.str();
  }

  ~ObserverError() throw() {}

  //! Observer prefix in observation/
  const std::string& prefix() const throw() { return prefix_; }

  //! Key that was not found
  const std::string& key() const throw() { return key_; }

 private:
  //! Observer prefix in observation/
  std::string prefix_;

  //! Key that was not found
  std::string key_;
};

}  // namespace upkie::cpp::exceptions
