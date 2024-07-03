// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <string>

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "upkie/cpp/actuation/ImuData.h"

namespace upkie::actuation {

//! IMU data with extra groundtruth observations from Bullet.
struct BulletImuData : public ImuData {
  //! Spatial linear velocity in [m] / [s]², used to compute the acceleration
  Eigen::Vector3d linear_velocity_imu_in_world = Eigen::Vector3d::Zero();
};

}  // namespace upkie::actuation
