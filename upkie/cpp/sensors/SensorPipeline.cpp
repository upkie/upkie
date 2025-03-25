// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Inria

#include "upkie/cpp/sensors/SensorPipeline.h"

#include <palimpsest/exceptions/KeyError.h>

namespace upkie::cpp::sensors {

using palimpsest::exceptions::KeyError;

void SensorPipeline::run(Dictionary& observation) {
  for (auto sensor : sensors_) {
    sensor->write(observation);
  }
}

}  // namespace upkie::cpp::sensors
