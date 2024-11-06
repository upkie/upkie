// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include <palimpsest/Dictionary.h>
#include <palimpsest/exceptions/KeyError.h>

#include <memory>

#include "gtest/gtest.h"
#include "upkie/cpp/observers/FloorContact.h"

namespace upkie::cpp::observers {

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
  observation("servo")("left_wheel")("velocity") = 0.0;
  ASSERT_NO_THROW(floor_contact_->read(observation));
}

TEST_F(FloorContactTest, NoTorqueNoContact) {
  floor_contact_->reset(config_);

  Dictionary observation;
  observation("servo")("left_wheel")("velocity") = 0.1;
  observation("servo")("left_wheel")("torque") = 0.0;
  observation("servo")("right_wheel")("velocity") = 0.1;
  observation("servo")("right_wheel")("torque") = 0.0;

  floor_contact_->read(observation);
  floor_contact_->write(observation);
  ASSERT_FALSE(observation("floor_contact").get<bool>("contact"));
}

TEST_F(FloorContactTest, BigWheelAccelTorqueMeansContact) {
  floor_contact_->reset(config_);

  Dictionary observation;

  const double vel = 10.0;  // rad/s
  observation("servo")("left_wheel")("velocity") = vel;
  observation("servo")("left_wheel")("torque") = 10.0;
  observation("servo")("right_wheel")("velocity") = vel;
  observation("servo")("right_wheel")("torque") = 10.0;
  floor_contact_->read(observation);

  // Increase velocity to produce accel > min_touchdown_acceleration
  const double new_vel = vel + 5.0 * dt_;
  observation("servo")("left_wheel")("velocity") = new_vel;
  observation("servo")("right_wheel")("velocity") = new_vel;
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
  observation("servo")("left_hip")("velocity") = 0.0;
  observation("servo")("left_hip")("velocity") = 0.0;
  observation("servo")("left_knee")("velocity") = 0.0;
  observation("servo")("left_knee")("velocity") = 0.0;
  observation("servo")("left_wheel")("torque") = 0.0;
  observation("servo")("left_wheel")("velocity") = 0.0;
  observation("servo")("right_hip")("torque") = 1.0;
  observation("servo")("right_hip")("torque") = 1.0;
  observation("servo")("right_knee")("torque") = 1.0;
  observation("servo")("right_knee")("torque") = 1.0;
  observation("servo")("right_wheel")("torque") = 0.0;
  observation("servo")("right_wheel")("velocity") = 0.0;

  floor_contact_->read(observation);
  floor_contact_->write(observation);
  ASSERT_FALSE(observation("floor_contact").get<bool>("contact"));
}

TEST_F(FloorContactTest, BigLegTorqueMeansContact) {
  floor_contact_->reset(config_);

  Dictionary observation;
  observation("servo")("left_hip")("torque") = 100.0;
  observation("servo")("left_hip")("velocity") = 0.0;
  observation("servo")("left_knee")("torque") = 100.0;
  observation("servo")("left_knee")("velocity") = 0.0;
  observation("servo")("left_wheel")("torque") = 0.0;
  observation("servo")("left_wheel")("velocity") = 0.0;
  observation("servo")("right_hip")("torque") = 100.0;
  observation("servo")("right_hip")("velocity") = 0.0;
  observation("servo")("right_knee")("torque") = 100.0;
  observation("servo")("right_knee")("velocity") = 0.0;
  observation("servo")("right_wheel")("torque") = 0.0;
  observation("servo")("right_wheel")("velocity") = 0.0;

  floor_contact_->read(observation);
  floor_contact_->write(observation);
  ASSERT_TRUE(observation("floor_contact").get<bool>("contact"));
}

}  // namespace upkie::cpp::observers
