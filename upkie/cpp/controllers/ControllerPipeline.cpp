// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Inria

#include "upkie/cpp/controllers/ControllerPipeline.h"

#include <palimpsest/exceptions/KeyError.h>

#include "upkie/cpp/exceptions/ControllerError.h"

namespace upkie::cpp::controllers {

using palimpsest::exceptions::KeyError;
using upkie::cpp::exceptions::ControllerError;

void ControllerPipeline::reset(const Dictionary& config) {
  for (auto controller : controllers_) {
    controller->reset(config);
  }
}

void ControllerPipeline::run(const Dictionary& observation,
                             Dictionary& action) {
  for (auto controller : controllers_) {
    try {
      controller->read(observation, action);
      controller->write(action);
    } catch (const std::exception& e) {
      spdlog::error("Controller {} threw an exception: {}", controller->name(),
                    e.what());
      throw ControllerError(controller->name(), e.what());
    }
  }
}

}  // namespace upkie::cpp::controllers
