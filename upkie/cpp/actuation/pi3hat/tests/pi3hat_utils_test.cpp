// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include <map>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "upkie/cpp/actuation/pi3hat/utils.h"

namespace upkie::cpp::actuation::pi3hat {

TEST(RawImuData, RawLinearAcceleration) {
  auto attitude = Eigen::Quaterniond::Identity();
  Eigen::Vector3d accel_mps2 = {0.1, 0.2, 0.0};
  Eigen::Vector3d imu_accel_mps2 =
      get_raw_linear_acceleration(attitude, accel_mps2);
  ASSERT_DOUBLE_EQ(imu_accel_mps2.x(), 0.1);
  ASSERT_DOUBLE_EQ(imu_accel_mps2.y(), 0.2);
  ASSERT_DOUBLE_EQ(imu_accel_mps2.z(), -9.81);
}

}  // namespace upkie::cpp::actuation::pi3hat
