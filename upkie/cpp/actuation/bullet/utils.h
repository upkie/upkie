// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "upkie/cpp/actuation/bullet/imu.h"

//! Bullet utility functions used in the simulation interface.
namespace upkie::cpp::actuation::bullet {

/*! Convert an Eigen quaternion to a Bullet one.
 *
 * \param[in] quat Eigen quaternion.
 *
 * \return Same quaternion for Bullet.
 */
inline btQuaternion bullet_from_eigen(const Eigen::Quaterniond& quat) {
  return btQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
}

/*! Convert an Eigen vector to a Bullet one.
 *
 * \param[in] v Eigen vector.
 *
 * \return Same vector for Bullet.
 */
inline btVector3 bullet_from_eigen(const Eigen::Vector3d& v) {
  return btVector3(v.x(), v.y(), v.z());
}

/*! Convert a Bullet quaternion to an Eigen one.
 *
 * \param[in] quat Bullet quaternion.
 *
 * \return Same vector for Eigen.
 */
inline Eigen::Quaterniond eigen_from_bullet(const btQuaternion& quat) {
  return Eigen::Quaterniond(quat.getW(), quat.getX(), quat.getY(), quat.getZ());
}

/*! Convert a Bullet vector to an Eigen one.
 *
 * \param[in] v Bullet vector.
 *
 * \return Same vector for Eigen.
 */
inline Eigen::Vector3d eigen_from_bullet(const btVector3& v) {
  return Eigen::Vector3d(v.getX(), v.getY(), v.getZ());
}

/*! Find the index of a link.
 *
 * \param[in] bullet Bullet client.
 * \param[in] robot Index of the robot to search.
 * \param[in] link_name Name of the searched link.
 *
 * \return Link index if found, -1 otherwise.
 */
inline int find_link_index(b3RobotSimulatorClientAPI& bullet, int robot,
                           const std::string& link_name) noexcept {
  b3JointInfo joint_info;
  int nb_joints = bullet.getNumJoints(robot);
  for (int joint_index = 0; joint_index < nb_joints; ++joint_index) {
    bullet.getJointInfo(robot, joint_index, &joint_info);
    if (std::string(joint_info.m_linkName) == link_name) {
      return joint_index;  // link and joint indices are the same in Bullet
    }
  }
  return -1;
}

/*! Get the position of a link frame in the world frame.
 *
 * \param[in] bullet Bullet client.
 * \param[in] robot Index of the robot to search.
 * \param[in] link_index Index of the link frame.
 *
 * \return Link index if found, -1 otherwise.
 *
 * \note This function will recompute forward kinematics.
 */
inline Eigen::Vector3d get_position_link_in_world(
    b3RobotSimulatorClientAPI& bullet, int robot, int link_index) noexcept {
  b3LinkState link_state;
  bullet.getLinkState(robot, link_index, /* computeVelocity = */ false,
                      /* computeForwardKinematics = */ true, &link_state);
  Eigen::Vector3d position_link_in_world = {
      link_state.m_worldLinkFramePosition[0],
      link_state.m_worldLinkFramePosition[1],
      link_state.m_worldLinkFramePosition[2],
  };
  return position_link_in_world;
}

/*! Get the total mass of the robot.
 *
 * \param[in] bullet Bullet client.
 * \param[in] robot Index of the robot.
 *
 * \return Total mass of the robot, in kilograms.
 */
inline double compute_robot_mass(b3RobotSimulatorClientAPI& bullet, int robot) {
  b3DynamicsInfo dynamics_info;
  double mass = 0;  // kg
  const int nb_joints = bullet.getNumJoints(robot);
  for (int joint_index = 0; joint_index < nb_joints; ++joint_index) {
    bullet.getDynamicsInfo(robot, joint_index, &dynamics_info);
    mass += dynamics_info.m_mass;
  }
  return mass;
}

/*! Get the position of the center of mass of a robot in the world frame.
 *
 * \param[in] bullet Bullet client.
 * \param[in] robot Index of the robot.
 *
 * \return Position of the center of mass in the world frame.
 *
 * \note This function will recompute forward kinematics.
 */
inline Eigen::Vector3d compute_position_com_in_world(
    b3RobotSimulatorClientAPI& bullet, int robot) {
  b3LinkState link_state;
  b3DynamicsInfo dynamics_info;
  double mass = 0.0;  // kg
  Eigen::Vector3d weighted_sum = Eigen::Vector3d::Zero();
  const int nb_joints = bullet.getNumJoints(robot);
  for (int link_index = 0; link_index < nb_joints; ++link_index) {
    bullet.getDynamicsInfo(robot, link_index, &dynamics_info);
    const double link_mass = dynamics_info.m_mass;
    Eigen::Vector3d position_link_com_in_link = {
        dynamics_info.m_localInertialFrame[0],
        dynamics_info.m_localInertialFrame[1],
        dynamics_info.m_localInertialFrame[2],
    };

    bullet.getLinkState(robot, link_index, /* computeVelocity = */ false,
                        /* computeForwardKinematics = */ true, &link_state);
    Eigen::Quaterniond orientation_link_in_world;
    orientation_link_in_world.w() = link_state.m_worldLinkFrameOrientation[3];
    orientation_link_in_world.x() = link_state.m_worldLinkFrameOrientation[0];
    orientation_link_in_world.y() = link_state.m_worldLinkFrameOrientation[1];
    orientation_link_in_world.z() = link_state.m_worldLinkFrameOrientation[2];
    Eigen::Vector3d position_link_in_world = {
        link_state.m_worldLinkFramePosition[0],
        link_state.m_worldLinkFramePosition[1],
        link_state.m_worldLinkFramePosition[2],
    };
    Eigen::Matrix3d rotation_link_to_world =
        orientation_link_in_world.toRotationMatrix();
    Eigen::Vector3d position_link_com_in_world =
        rotation_link_to_world * position_link_com_in_link +
        position_link_in_world;

    weighted_sum += link_mass * position_link_com_in_world;
    mass += link_mass;
  }
  return weighted_sum / mass;
}

}  // namespace upkie::cpp::actuation::bullet
