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

  // Pitch
  auto quat_imu_in_ars =
      observation("imu").get<Eigen::Quaterniond>("orientation");
  pitch_base_in_world_ =
      compute_base_pitch_from_imu(quat_imu_in_ars, params_.rotation_base_to_imu,
                                  params_.rotation_ars_to_world);

  // Angular velocity
  auto angular_velocity_imu_in_imu =
      observation("imu").get<Eigen::Vector3d>("angular_velocity");
  angular_velocity_base_in_base_ = compute_base_angular_velocity_from_imu(
      angular_velocity_imu_in_imu, params_.rotation_base_to_imu);
}

void BaseOrientation::write(Dictionary& observation) {
  auto& output = observation(prefix());
  output("pitch") = pitch_base_in_world_;
  output("angular_velocity") = angular_velocity_base_in_base_;
}

}  // namespace upkie::observers
