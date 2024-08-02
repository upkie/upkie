// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "upkie/cpp/actuation/pi3hat/imu_eigen.h"

namespace upkie::cpp::actuation::pi3hat {

TEST(IMU, RawAngularVelocity) {
  Eigen::Vector3d rate_dps = {180.0, -90.0, -0.0};
  Eigen::Vector3d bias_dps = {90.0, 90.0, 90.0};
  Eigen::Vector3d gyro = get_raw_angular_velocity(rate_dps, bias_dps);
  ASSERT_NEAR(gyro.x(), M_PI / 2, 1e-12);
  ASSERT_NEAR(gyro.y(), -M_PI, 1e-12);
  ASSERT_NEAR(gyro.z(), -M_PI / 2, 1e-12);
}

TEST(IMU, RawLinearAccelerationAtRest) {
  // At rest, with the Upkie in its zero configuration, the IMU frame is
  // oriented with x to the rear of the robot, y to its left and z downward.
  auto orientation_imu_in_ars = Eigen::Quaterniond::Identity();
  auto accel_mps2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d imu_accel_mps2 =
      get_raw_linear_acceleration(orientation_imu_in_ars, accel_mps2);

  // Raw acceleration should be accel_imu - gravity_imu = [0 0 -g]
  // Note: gravity_imu = [0 0 +g] as the z-axis of the IMU frame is downward
  ASSERT_NEAR(imu_accel_mps2.x(), 0.0, 1e-12);
  ASSERT_NEAR(imu_accel_mps2.y(), 0.0, 1e-12);
  ASSERT_NEAR(imu_accel_mps2.z(), -kMjbotsGravity, 1e-12);
}

TEST(IMU, RawLinearAccelerationQuarterXTurn) {
  Eigen::Quaterniond orientation_imu_in_ars(
      Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitX()));
  Eigen::Vector3d accel_mps2 = {0.1, 0.0, 0.2};
  Eigen::Vector3d imu_accel_mps2 =
      get_raw_linear_acceleration(orientation_imu_in_ars, accel_mps2);
  ASSERT_NEAR(imu_accel_mps2.x(), 0.1, 1e-12);
  ASSERT_NEAR(imu_accel_mps2.y(), kMjbotsGravity, 1e-12);
  ASSERT_NEAR(imu_accel_mps2.z(), 0.2, 1e-12);
}

}  // namespace upkie::cpp::actuation::pi3hat
