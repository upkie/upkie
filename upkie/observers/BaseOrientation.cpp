// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#include "upkie/observers/BaseOrientation.h"

namespace upkie::observers {

BaseOrientation::BaseOrientation(const Parameters& params) : params_(params) {}

void BaseOrientation::reset(const Dictionary& config) {
  params_.configure(config);
}

void BaseOrientation::read(const Dictionary& observation) {
  if (!observation.has("imu")) {
    return;
  }
  auto quat_imu_in_ars =
      observation("imu").get<Eigen::Quaterniond>("orientation");
  pitch_base_in_world_ = compute_base_pitch_from_imu(quat_imu_in_ars);
}

void BaseOrientation::write(Dictionary& observation) {
  auto& output = observation(prefix());
  output("pitch") = pitch_base_in_world_;
}

}  // namespace upkie::observers
