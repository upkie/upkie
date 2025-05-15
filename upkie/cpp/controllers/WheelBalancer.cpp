// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Inria

#include "upkie/cpp/controllers/WheelBalancer.h"

namespace upkie::cpp::controllers {

WheelBalancer::WheelBalancer(const Parameters& params) : params_(params) {}

void WheelBalancer::reset(const Dictionary& config) {
  params_.configure(config);
}

void WheelBalancer::read(const Dictionary& observation,
                         const Dictionary& action) {}

void WheelBalancer::write(Dictionary& action) {}

}  // namespace upkie::cpp::controllers
