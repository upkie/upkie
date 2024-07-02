// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <string>

namespace upkie {

//! Exception thrown when an observer fails.
class ObserverError : public std::runtime_error {
 public:
  /*! Create an observer error.
   *
   * \param[in] prefix Observer prefix.
   * \param[in] key Key that was not found.
   */
  ObserverError(const std::string& prefix, const std::string& key)
      : std::runtime_error(key), prefix_(prefix), key_(key) {
    std::ostringstream out;
    out << "Failed to update /observation/" << prefix << ": key \"" << key
        << "\" not found";
    message_ = out.str();
  }

  ~ObserverError() throw() {}

  //! Error message
  const char* what() const throw() { return message_.c_str(); }

  //! Observer prefix in observation/
  const std::string& prefix() const throw() { return prefix_; }

  //! Key that was not found
  const std::string& key() const throw() { return key_; }

 private:
  //! Observer prefix in observation/
  std::string prefix_;

  //! Key that was not found
  std::string key_;

  //! Complete error message
  std::string message_;
};

}  // namespace upkie
