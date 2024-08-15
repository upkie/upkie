// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include <map>
#include <memory>
#include <string>

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "gtest/gtest.h"
#include "tools/cpp/runfiles/runfiles.h"
#include "upkie/cpp/actuation/ImuData.h"
#include "upkie/cpp/actuation/bullet/utils.h"

using bazel::tools::cpp::runfiles::Runfiles;

namespace upkie::cpp::actuation::bullet {

class BulletIMUTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string error;
    std::unique_ptr<Runfiles> runfiles(Runfiles::CreateForTest(&error));
    ASSERT_NE(runfiles, nullptr);

    std::string urdf_path =
        runfiles->Rlocation("upkie_description/urdf/upkie.urdf");

    bullet_ = std::make_unique<b3RobotSimulatorClientAPI>();
    bool is_connected = bullet_->connect(eCONNECT_DIRECT);
    ASSERT_TRUE(is_connected);
    robot_ = bullet_->loadURDF(urdf_path);
  }

 protected:
  //! Bullet client
  std::unique_ptr<b3RobotSimulatorClientAPI> bullet_;

  //! Robot identifier
  int robot_;
};

TEST_F(BulletIMUTest, IMULinearAccelerationSeesGravityInFreeFall) {
  const int imu_link_index = find_link_index(*bullet_, robot_, "imu");
  const double dt = 1. / 240.;  // [s]

  // Remove simulator damping to get actual free fall physics
  b3RobotSimulatorChangeDynamicsArgs change_dynamics_args;
  change_dynamics_args.m_linearDamping = 0.0;
  change_dynamics_args.m_angularDamping = 0.0;
  bullet_->changeDynamics(robot_, -1, change_dynamics_args);
  bullet_->setGravity(btVector3(0, 0, -4.2));
  bullet_->setTimeStep(dt);

  // Rotate the base so that the IMU frame has its x-axis downward
  auto position_base_in_world = btVector3(0., 0., 0.);
  auto orientation_base_in_world =
      bullet_->getQuaternionFromEuler(btVector3(0., -M_PI / 2, 0.));
  bullet_->resetBasePositionAndOrientation(robot_, position_base_in_world,
                                           orientation_base_in_world);

  ImuData imu_data;
  read_imu_data(imu_data, *bullet_, robot_, imu_link_index, dt);
  bullet_->stepSimulation();
  read_imu_data(imu_data, *bullet_, robot_, imu_link_index, dt);
  ASSERT_NEAR(imu_data.linear_acceleration_imu_in_imu.x(), 4.2, 1e-10);
  ASSERT_NEAR(imu_data.linear_acceleration_imu_in_imu.y(), 0.0, 1e-10);
  ASSERT_NEAR(imu_data.linear_acceleration_imu_in_imu.z(), 0.0, 1e-10);
}

TEST_F(BulletIMUTest, RawIMULinearAcceleration) {
  const int imu_link_index = find_link_index(*bullet_, robot_, "imu");
  const double dt = 1. / 220.;  // [s]

  // Remove simulator damping to get actual free fall physics
  b3RobotSimulatorChangeDynamicsArgs change_dynamics_args;
  change_dynamics_args.m_linearDamping = 0.0;
  change_dynamics_args.m_angularDamping = 0.0;
  bullet_->changeDynamics(robot_, -1, change_dynamics_args);
  bullet_->setGravity(btVector3(0, 0, -kGravity));
  bullet_->setTimeStep(dt);

  // Rotate the base so that the IMU frame has its x-axis downward
  auto position_base_in_world = btVector3(0., 0., 0.);
  auto orientation_base_in_world =
      bullet_->getQuaternionFromEuler(btVector3(0., -M_PI / 2, 0.));
  bullet_->resetBasePositionAndOrientation(robot_, position_base_in_world,
                                           orientation_base_in_world);

  ImuData imu_data;
  read_imu_data(imu_data, *bullet_, robot_, imu_link_index, dt);
  bullet_->stepSimulation();
  read_imu_data(imu_data, *bullet_, robot_, imu_link_index, dt);

  // Raw measurement is the proper acceleration
  ASSERT_NEAR(imu_data.raw_linear_acceleration.x(), 0.0, 1e-10);
  ASSERT_NEAR(imu_data.raw_linear_acceleration.y(), 0.0, 1e-10);
  ASSERT_NEAR(imu_data.raw_linear_acceleration.z(), 0.0, 1e-10);

  // Filtered acceleration substracts gravity
  ASSERT_NEAR(imu_data.linear_acceleration_imu_in_imu.x(), kGravity, 1e-10);
  ASSERT_NEAR(imu_data.linear_acceleration_imu_in_imu.y(), 0.0, 1e-10);
  ASSERT_NEAR(imu_data.linear_acceleration_imu_in_imu.z(), 0.0, 1e-10);
}

}  // namespace upkie::cpp::actuation::bullet
