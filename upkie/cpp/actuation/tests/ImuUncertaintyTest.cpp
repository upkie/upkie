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

TEST_F(ImuUncertaintyTest, NoNoise) {
  ImuUncertainty imu_uncertainty;
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  imu_uncertainty.apply(linear_acceleration, angular_velocity, *rng_);
  ASSERT_LT(linear_acceleration.norm(), 1e-10);
  ASSERT_LT(angular_velocity.norm(), 1e-10);
}

}  // namespace upkie::cpp::actuation
