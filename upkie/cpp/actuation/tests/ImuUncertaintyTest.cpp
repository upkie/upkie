// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#include <vector>

#include "gtest/gtest.h"
#include "upkie/cpp/actuation/ImuUncertainty.h"

namespace upkie::cpp::actuation {

class ImuUncertaintyTest : public ::testing::Test {
 protected:
  //! Set up a new test fixture
  void SetUp() override {
    rng_ = std::make_unique<std::mt19937>(std::random_device()());
  }

 protected:
  //! Random number generator used to sample from probability distributions
  std::unique_ptr<std::mt19937> rng_;
};

TEST_F(ImuUncertaintyTest, NoBiasNoNoise) {
  ImuUncertainty imu_uncertainty;
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  imu_uncertainty.apply(linear_acceleration, angular_velocity, *rng_);
  ASSERT_LT(linear_acceleration.norm(), 1e-10);
  ASSERT_LT(angular_velocity.norm(), 1e-10);
}

TEST_F(ImuUncertaintyTest, PureBias) {
  ImuUncertainty imu_uncertainty;
  imu_uncertainty.accelerometer_bias = Eigen::Vector3d{1.0, 2.0, 3.0};
  imu_uncertainty.gyroscope_bias = Eigen::Vector3d{0.1, 0.2, 0.2};
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  imu_uncertainty.apply(linear_acceleration, angular_velocity, *rng_);
  ASSERT_LT((linear_acceleration - imu_uncertainty.accelerometer_bias).norm(),
            1e-10);
  ASSERT_LT((angular_velocity - imu_uncertainty.gyroscope_bias).norm(), 1e-10);
}

}  // namespace upkie::cpp::actuation
