// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/cpp/actuation/bullet/utils.h"

#include <map>
#include <memory>
#include <string>

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "gtest/gtest.h"
#include "tools/cpp/runfiles/runfiles.h"

using bazel::tools::cpp::runfiles::Runfiles;

namespace upkie::cpp::actuation::bullet {

class BulletUtilsTest : public ::testing::Test {
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

TEST_F(BulletUtilsTest, WheelContactsHaveStiffnessDamping) {
  b3DynamicsInfo info;
  b3JointInfo joint_info;
  int nb_joints = bullet_->getNumJoints(robot_);
  for (int joint_index = 0; joint_index < nb_joints; ++joint_index) {
    bullet_->getJointInfo(robot_, joint_index, &joint_info);
    bullet_->getDynamicsInfo(robot_, joint_index, &info);
    std::string link_name = joint_info.m_linkName;
    if (link_name == "left_wheel_tire" || link_name == "right_wheel_tire") {
      ASSERT_GT(info.m_contactStiffness, 1000.0);
      ASSERT_GT(info.m_contactDamping, 100.0);
    } else {
      ASSERT_DOUBLE_EQ(info.m_contactStiffness, -1.0);
      ASSERT_DOUBLE_EQ(info.m_contactDamping, -1.0);
    }
  }
}

TEST_F(BulletUtilsTest, FindLinkIndex) {
  ASSERT_GE(find_link_index(*bullet_, robot_, "imu"), 0);
  ASSERT_EQ(find_link_index(*bullet_, robot_, "umi"), -1);
}

TEST_F(BulletUtilsTest, BulletFromEigenVector) {
  Eigen::Vector3d eigen_vec(1., 2., 3.);
  auto bullet_vec = bullet_from_eigen(eigen_vec);
  ASSERT_DOUBLE_EQ(bullet_vec.getX(), eigen_vec.x());
  ASSERT_DOUBLE_EQ(bullet_vec.getY(), eigen_vec.y());
  ASSERT_DOUBLE_EQ(bullet_vec.getZ(), eigen_vec.z());
}

TEST_F(BulletUtilsTest, EigenFromBulletVector) {
  btVector3 bullet_vec(1., 2., 3.);
  auto eigen_vec = eigen_from_bullet(bullet_vec);
  ASSERT_DOUBLE_EQ(eigen_vec.x(), bullet_vec.getX());
  ASSERT_DOUBLE_EQ(eigen_vec.y(), bullet_vec.getY());
  ASSERT_DOUBLE_EQ(eigen_vec.z(), bullet_vec.getZ());
}

TEST_F(BulletUtilsTest, EigenFromBulletQuat) {
  btQuaternion bullet_quat(btVector3(1., 0., 0.), 0.7);
  auto eigen_quat = eigen_from_bullet(bullet_quat);
  ASSERT_DOUBLE_EQ(eigen_quat.w(), bullet_quat.getW());
  ASSERT_DOUBLE_EQ(eigen_quat.x(), bullet_quat.getX());
  ASSERT_DOUBLE_EQ(eigen_quat.y(), bullet_quat.getY());
  ASSERT_DOUBLE_EQ(eigen_quat.z(), bullet_quat.getZ());
}

TEST_F(BulletUtilsTest, RobotMass) {
  ASSERT_NEAR(compute_robot_mass(*bullet_, robot_), 5.3382, 1e-4);
}

TEST_F(BulletUtilsTest, ComputeCenterOfMass) {
  Eigen::Vector3d com = compute_position_com_in_world(*bullet_, robot_);
  ASSERT_NEAR(com.x(), -0.0059, 1e-4);
  ASSERT_NEAR(com.y(), 0.0, 1e-4);
  ASSERT_NEAR(com.z(), -0.2455, 1e-4);
}

}  // namespace upkie::cpp::actuation::bullet
