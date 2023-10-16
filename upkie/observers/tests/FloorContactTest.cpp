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
#include "upkie/observers/FloorContact.h"

namespace upkie::observers::tests {

using palimpsest::Dictionary;
using palimpsest::exceptions::KeyError;

class FloorContactTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_("floor_contact")("upper_leg_torque_threshold") = 10.0;
    config_("wheel_contact")("liftoff_inertia") = 0.001;
    config_("wheel_contact")("min_touchdown_acceleration") = 2.0;
    config_("wheel_contact")("min_touchdown_torque") = 0.015;
    config_("wheel_contact")("touchdown_inertia") = 0.004;

    // We set the cutoff period close to dt_ to generate accelerations larger
    // than the min touchdown acceleration more easily in some tests.
    config_("wheel_contact")("cutoff_period") = 3 * dt_;

    FloorContact::Parameters params;
    params.dt = dt_;
    params.upper_leg_joints = {"hip"};
    params.wheels = {"big_wheel", "small_wheel"};

    floor_contact_ = std::make_unique<FloorContact>(params);
  }

 protected:
  const double dt_ = 1.0 / 250.0;

  //! Configuration
  Dictionary config_;

  //! Observer
  std::unique_ptr<FloorContact> floor_contact_;
};

TEST_F(FloorContactTest, WriteEvenAfterEmptyRead) {
  // Writing after an empty read is the current spec, though not ideal
  Dictionary observation;
  floor_contact_->read(observation);
  floor_contact_->write(observation);
  ASSERT_FALSE(observation.is_empty());
}

TEST_F(FloorContactTest, UnconfiguredDoesNotThrow) {
  FloorContact::Parameters params;
  FloorContact observer(params);

  Dictionary empty_config;
  observer.reset(empty_config);

  Dictionary observation;
  ASSERT_NO_THROW(observer.read(observation));
}

TEST_F(FloorContactTest, PartialObservationDoesNotThrow) {
  Dictionary observation;
  observation("servo")("big_wheel")("velocity") = 0.0;
  ASSERT_NO_THROW(floor_contact_->read(observation));
}

TEST_F(FloorContactTest, NoTorqueNoContact) {
  floor_contact_->reset(config_);

  Dictionary observation;
  observation("servo")("big_wheel")("velocity") = 0.1;
  observation("servo")("big_wheel")("torque") = 0.0;
  observation("servo")("small_wheel")("velocity") = 0.1;
  observation("servo")("small_wheel")("torque") = 0.0;

  floor_contact_->read(observation);
  floor_contact_->write(observation);
  ASSERT_FALSE(observation("floor_contact").get<bool>("contact"));
}

TEST_F(FloorContactTest, BigWheelAccelTorqueMeansContact) {
  floor_contact_->reset(config_);

  Dictionary observation;

  const double vel = 10.0;  // rad/s
  observation("servo")("big_wheel")("velocity") = vel;
  observation("servo")("big_wheel")("torque") = 10.0;
  observation("servo")("small_wheel")("velocity") = vel;
  observation("servo")("small_wheel")("torque") = 10.0;
  floor_contact_->read(observation);

  // Increase velocity to produce accel > min_touchdown_acceleration
  const double new_vel = vel + 5.0 * dt_;
  observation("servo")("big_wheel")("velocity") = new_vel;
  observation("servo")("small_wheel")("velocity") = new_vel;
  floor_contact_->read(observation);

  floor_contact_->write(observation);
  ASSERT_TRUE(observation("floor_contact").get<bool>("contact"));

  // Extra test: joystick button resets wheel observers
  observation("joystick")("cross_button") = true;
  floor_contact_->read(observation);
  floor_contact_->write(observation);
  // The following observation would be true without a reset
  ASSERT_FALSE(observation("floor_contact").get<bool>("contact"));
}

TEST_F(FloorContactTest, SmallLegTorqueNoContact) {
  floor_contact_->reset(config_);

  Dictionary observation;
  observation("servo")("big_wheel")("velocity") = 0.0;
  observation("servo")("big_wheel")("torque") = 0.0;
  observation("servo")("small_wheel")("velocity") = 0.0;
  observation("servo")("small_wheel")("torque") = 0.0;
  observation("servo")("hip")("velocity") = 0.0;
  observation("servo")("hip")("torque") = 1.0;

  floor_contact_->read(observation);
  floor_contact_->write(observation);
  ASSERT_FALSE(observation("floor_contact").get<bool>("contact"));
}

TEST_F(FloorContactTest, BigLegTorqueMeansContact) {
  floor_contact_->reset(config_);

  Dictionary observation;
  observation("servo")("big_wheel")("velocity") = 0.0;
  observation("servo")("big_wheel")("torque") = 0.0;
  observation("servo")("small_wheel")("velocity") = 0.0;
  observation("servo")("small_wheel")("torque") = 0.0;
  observation("servo")("hip")("velocity") = 0.0;
  observation("servo")("hip")("torque") = 100.0;

  floor_contact_->read(observation);
  floor_contact_->write(observation);
  ASSERT_TRUE(observation("floor_contact").get<bool>("contact"));
}

}  // namespace upkie::observers::tests
