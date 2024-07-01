// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>

#include "upkie/cpp/observers/Observer.h"

namespace upkie {

//! Exception-throwing observer
class ThrowingObserver : public Observer {
 public:
  void read(const palimpsest::Dictionary& observation) override {
    throw std::runtime_error("could not get schwifty");
  }
};

}  // namespace upkie
