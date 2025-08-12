// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Stéphane Caron

#pragma once

#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "upkie/cpp/interfaces/ImuData.h"
#include "upkie/cpp/interfaces/bullet/gravity.h"

namespace upkie::cpp::interfaces::bullet {

/*! Compute groundtruth IMU quantities from the IMU link state.
 *
 * \param[out] imu_data IMU data to update.
 * \param[in] bullet Bullet client.
 * \param[in] robot Bullet index of the robot model.
 * \param[in] imu_link_index Index of the IMU link in the robot.
 * \param[in] dt Simulation timestep in [s].
 */
inline void read_imu_data(ImuData& imu_data, b3RobotSimulatorClientAPI& bullet,
                          int robot, const int imu_link_index, double dt) {
  b3LinkState link_state;
  bullet.getLinkState(robot, imu_link_index, /* computeVelocity = */ true,
                      /* computeForwardKinematics = */ true, &link_state);

  Eigen::Quaterniond orientation_imu_in_world;
  orientation_imu_in_world.w() = link_state.m_worldLinkFrameOrientation[3];
  orientation_imu_in_world.x() = link_state.m_worldLinkFrameOrientation[0];
  orientation_imu_in_world.y() = link_state.m_worldLinkFrameOrientation[1];
  orientation_imu_in_world.z() = link_state.m_worldLinkFrameOrientation[2];

  // The attitude reference system frame has +x forward, +y right and +z down,
  // whereas our world frame has +x forward, +y left and +z up:
  // https://github.com/mjbots/pi3hat/blob/master/docs/reference.md#orientation
  Eigen::Matrix3d rotation_world_to_ars =
      Eigen::Vector3d{1.0, -1.0, -1.0}.asDiagonal();

  Eigen::Matrix3d rotation_imu_to_world =
      orientation_imu_in_world.toRotationMatrix();
  Eigen::Matrix3d rotation_imu_to_ars =
      rotation_world_to_ars * rotation_imu_to_world;
  Eigen::Quaterniond orientation_imu_in_ars(rotation_imu_to_ars);

  // Extract velocities
  Eigen::Vector3d linear_velocity_imu_in_world = {
      link_state.m_worldLinearVelocity[0],
      link_state.m_worldLinearVelocity[1],
      link_state.m_worldLinearVelocity[2],
  };
  Eigen::Vector3d angular_velocity_imu_to_world_in_world = {
      link_state.m_worldAngularVelocity[0],
      link_state.m_worldAngularVelocity[1],
      link_state.m_worldAngularVelocity[2],
  };

  // Compute linear acceleration in the world frame by discrete differentiation
  const auto& previous_linear_velocity = imu_data.linear_velocity_imu_in_world;
  Eigen::Vector3d linear_acceleration_imu_in_world =
      (linear_velocity_imu_in_world - previous_linear_velocity) / dt;

  auto rotation_world_to_imu = orientation_imu_in_world.normalized().inverse();
  Eigen::Vector3d angular_velocity_imu_in_imu =
      rotation_world_to_imu * angular_velocity_imu_to_world_in_world;

  // Accelerometer readings (before and after filtering)
  Eigen::Vector3d linear_acceleration_imu_in_imu =
      rotation_world_to_imu * linear_acceleration_imu_in_world;
  Eigen::Vector3d gravity_in_world = {0., 0., -9.81};
  Eigen::Vector3d proper_acceleration_in_imu =
      rotation_world_to_imu *
      (linear_acceleration_imu_in_world - gravity_in_world);

  // UKF outputs
  imu_data.orientation_imu_in_ars = orientation_imu_in_ars;
  imu_data.angular_velocity_imu_in_imu = angular_velocity_imu_in_imu;
  imu_data.linear_acceleration_imu_in_imu = linear_acceleration_imu_in_imu;

  // Raw measurements
  imu_data.raw_angular_velocity = angular_velocity_imu_in_imu;
  imu_data.raw_linear_acceleration = proper_acceleration_in_imu;

  // ... and the extra field for the Bullet interface
  imu_data.linear_velocity_imu_in_world = linear_velocity_imu_in_world;
}

}  // namespace upkie::cpp::interfaces::bullet
