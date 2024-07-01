// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/cpp/observers/ObserverPipeline.h"

#include <palimpsest/exceptions/KeyError.h>

#include "upkie/cpp/exceptions/ObserverError.h"

namespace upkie::cpp::observation {

using palimpsest::exceptions::KeyError;
using upkie::cpp::exceptions::ObserverError;

void ObserverPipeline::reset(const Dictionary& config) {
  for (auto observer : observers_) {
    observer->reset(config);
  }
}

void ObserverPipeline::run(Dictionary& observation) {
  for (auto sensor : sensors_) {
    sensor->write(observation);
  }
  for (auto observer : observers_) {
    try {
      observer->read(observation);
      observer->write(observation);
    } catch (const KeyError& e) {
      throw ObserverError(observer->prefix(), e.key());
    } catch (const std::exception& e) {
      spdlog::error("[ObserverPipeline] Observer {} threw an exception: {}",
                    observer->prefix(), e.what());
      throw;
    }
  }
}

}  // namespace upkie::cpp::observation
