// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>
#include <palimpsest/exceptions/KeyError.h>

#include "upkie/cpp/observation/Observer.h"

namespace upkie::cpp::observation::tests {

using palimpsest::exceptions::KeyError;

//! An observer that gets schwifty
class SchwiftyObserver : public observation::Observer {
 public:
  void write(palimpsest::Dictionary& observation) override {
    observation("schwifty") = true;
    if (throw_exception) {
      throw std::runtime_error("could not get schwifty");
    } else if (throw_key_error) {
      throw KeyError("schwift", __FILE__, __LINE__, "");
    }
  }

  //! Throw a runtime error
  bool throw_exception = false;

  //! Throw a KeyError
  bool throw_key_error = false;
};

}  // namespace upkie::cpp::observation::tests
