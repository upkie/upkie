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
    BaseOrientation::Parameters params;
    base_orientation_ = std::make_unique<BaseOrientation>(params);
  }

 protected:
  //! Observer
  std::unique_ptr<BaseOrientation> base_orientation_;
};

TEST_F(BaseOrientationTest, Lateral) {
  Eigen::Quaterniond quat_imu_in_ars = {
      0.008472769239730098,   // w
      -0.9953038144146671,    // x
      -0.09639792825405252,   // y
      -0.002443076206500708,  // z
  };
  Eigen::Matrix3d rotation_base_to_imu;
  rotation_base_to_imu << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  double pitch_base_in_world =
      compute_base_pitch_from_imu(quat_imu_in_ars, rotation_base_to_imu)
          ASSERT_NEAR(pitch_base_in_world, -0.016, 1e-3);
}

}  // namespace upkie::observers::tests
