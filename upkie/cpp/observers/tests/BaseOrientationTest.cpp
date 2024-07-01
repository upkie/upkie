// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#include <palimpsest/Dictionary.h>
#include <palimpsest/exceptions/KeyError.h>

#include <memory>

#include "gtest/gtest.h"
#include "upkie/observers/BaseOrientation.h"

namespace upkie::observers::tests {

using palimpsest::Dictionary;
using palimpsest::exceptions::KeyError;

class BaseOrientationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rotation_ars_to_world_.setZero();
    rotation_ars_to_world_.diagonal() << 1.0, -1.0, -1.0;

    BaseOrientation::Parameters params;
    base_orientation_ = std::make_unique<BaseOrientation>(params);
  }

 protected:
  //! Observer
  std::unique_ptr<BaseOrientation> base_orientation_;

  //! Default rotation from ARS to world
  Eigen::Matrix3d rotation_ars_to_world_;
};

// Check that a pure rotation around the z-axis yields no pitch.
TEST_F(BaseOrientationTest, ZeroPitch) {
  const double phi = 0.42;
  Eigen::Matrix3d orientation_imu_in_world;
  orientation_imu_in_world << cos(phi), -sin(phi), 0.0, sin(phi), cos(phi), 0.0,
      0.0, 0.0, 1.0;
  double imu_pitch = compute_pitch_frame_in_parent(orientation_imu_in_world);
  ASSERT_DOUBLE_EQ(imu_pitch, 0.0);
}

// Check that a pure rotation around the y-axis yields the correct pitch.
TEST_F(BaseOrientationTest, CloseToZero) {
  const double theta = 1e-3;
  Eigen::Matrix3d orientation_imu_in_world;
  orientation_imu_in_world << cos(theta), 0.0, sin(theta), 0.0, 1.0, 0.0,
      -sin(theta), 0.0, cos(theta);
  double imu_pitch = compute_pitch_frame_in_parent(orientation_imu_in_world);
  ASSERT_NEAR(imu_pitch, theta, 1e-6);
}

// As an extra, the function can handle a partially-normalized input. We
// are going the extra mile here, this is not strictly necessary.
TEST_F(BaseOrientationTest, OrientationNotNeatlyNormalized) {
  const double theta = 1e-3;
  Eigen::Matrix3d orientation_imu_in_world;
  orientation_imu_in_world << cos(theta), 0.0, sin(theta), 0.0, 1.0, 0.0,
      -sin(theta), 0.0, cos(theta);
  orientation_imu_in_world.col(0) *= 1.0 - 1e-2;
  double imu_pitch = compute_pitch_frame_in_parent(orientation_imu_in_world);
  ASSERT_NEAR(imu_pitch, theta, 1e-6);
}

TEST_F(BaseOrientationTest, BasePitchFromIMU) {
  Eigen::Quaterniond quat_imu_in_ars = {
      0.008472769239730098,   // w
      -0.9953038144146671,    // x
      -0.09639792825405252,   // y
      -0.002443076206500708,  // z
  };

  Eigen::Matrix3d rotation_base_to_imu;
  rotation_base_to_imu << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  double pitch_base_in_world = compute_base_pitch_from_imu(
      quat_imu_in_ars, rotation_base_to_imu, rotation_ars_to_world_);
  ASSERT_NEAR(pitch_base_in_world, -0.016, 1e-3);
}

TEST_F(BaseOrientationTest, PartialIMUObservation) {
  Dictionary observation;
  observation("imu").insert<Eigen::Quaterniond>("orientation",
                                                Eigen::Quaterniond::Identity());
  ASSERT_THROW(base_orientation_->read(observation), KeyError);
}

TEST_F(BaseOrientationTest, NeutralValues) {
  Dictionary observation;
  auto quat_imu_in_ars = Eigen::Quaterniond::Identity();
  observation("imu").insert<Eigen::Quaterniond>("orientation", quat_imu_in_ars);
  observation("imu").insert<Eigen::Vector3d>("angular_velocity",
                                             Eigen::Vector3d::Zero());
  base_orientation_->read(observation);
  base_orientation_->write(observation);

  // The angle is -pi/2 because the identity is the rotation from IMU to ARS,
  // not from base to world (check out parameters for details)
  ASSERT_DOUBLE_EQ(observation("base_orientation").get<double>("pitch"),
                   -0.5 * M_PI);

  auto angular_velocity_base_in_base =
      observation("base_orientation").get<Eigen::Vector3d>("angular_velocity");
  ASSERT_DOUBLE_EQ(angular_velocity_base_in_base.x(), 0.0);
  ASSERT_DOUBLE_EQ(angular_velocity_base_in_base.y(), 0.0);
  ASSERT_DOUBLE_EQ(angular_velocity_base_in_base.z(), 0.0);
}

}  // namespace upkie::observers::tests
