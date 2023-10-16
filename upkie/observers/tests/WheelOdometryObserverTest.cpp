/*
 * Copyright 2022 Stéphane Caron
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <palimpsest/Dictionary.h>
#include <palimpsest/exceptions/KeyError.h>

#include <memory>

#include "gtest/gtest.h"
#include "upkie/observers/WheelOdometry.h"

namespace upkie::observers::tests {

using palimpsest::Dictionary;
using palimpsest::exceptions::KeyError;

class WheelOdometryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_("wheel_odometry")("signed_radius")("left_wheel") = 0.50;
    config_("wheel_odometry")("signed_radius")("right_wheel") = -0.50;

    WheelOdometry::Parameters params;
    params.dt = 1.0 / 1000u;
    params.configure(config_);
    odom_ = std::make_unique<WheelOdometry>(params);
  }

 protected:
  //! Test config
  Dictionary config_;

  //! Observer
  std::unique_ptr<WheelOdometry> odom_;
};

TEST_F(WheelOdometryTest, WaitForContact) {
  Dictionary observation;
  ASSERT_THROW(odom_->read(observation), KeyError);
}

TEST_F(WheelOdometryTest, Write) {
  Dictionary observation;
  odom_->write(observation);
  ASSERT_TRUE(observation.has("wheel_odometry"));
}

TEST_F(WheelOdometryTest, GoForward) {
  Dictionary observation;
  observation("floor_contact")("left_wheel")("contact") = true;
  observation("floor_contact")("right_wheel")("contact") = true;
  observation("floor_contact")("contact") = true;
  observation("servo")("left_wheel")("velocity") = 1.0;
  observation("servo")("right_wheel")("velocity") = -1.0;

  odom_->read(observation);
  odom_->write(observation);
  const double radius =
      config_("wheel_odometry")("signed_radius")("left_wheel");
  ASSERT_DOUBLE_EQ(observation("wheel_odometry")("velocity"), radius * 1.0);
}

TEST_F(WheelOdometryTest, TurnInPlace) {
  Dictionary observation;
  observation("floor_contact")("left_wheel")("contact") = true;
  observation("floor_contact")("right_wheel")("contact") = true;
  observation("floor_contact")("contact") = true;
  observation("servo")("left_wheel")("velocity") = 1.0;
  observation("servo")("right_wheel")("velocity") = 1.0;

  odom_->read(observation);
  odom_->write(observation);
  ASSERT_DOUBLE_EQ(observation("wheel_odometry")("velocity"), 0.0);
}

TEST_F(WheelOdometryTest, ZeroVelocityWhenNoContact) {
  Dictionary observation;
  observation("floor_contact")("left_wheel")("contact") = false;
  observation("floor_contact")("right_wheel")("contact") = true;
  observation("floor_contact")("contact") = false;
  observation("servo")("left_wheel")("velocity") = 1.0;
  observation("servo")("right_wheel")("velocity") = 1.0;

  odom_->read(observation);
  odom_->write(observation);
  ASSERT_DOUBLE_EQ(observation("wheel_odometry")("position"), 0.0);
  ASSERT_DOUBLE_EQ(observation("wheel_odometry")("velocity"), 0.0);
}

}  // namespace upkie::observers::tests
