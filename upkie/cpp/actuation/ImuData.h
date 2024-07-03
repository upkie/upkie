// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace upkie::actuation {

//! Data filtered from an onboard IMU such as the pi3hat's.
struct ImuData {
  /*! Orientation from the IMU frame to the attitude reference system (ARS)
   * frame.
   *
   * The attitude reference system frame has +x forward, +y right and +z down,
   * whereas our world frame has +x forward, +y left and +z up:
   * https://github.com/mjbots/pi3hat/blob/ab632c82bd501b9fcb6f8200df0551989292b7a1/docs/reference.md#orientation
   */
  Eigen::Quaterniond orientation_imu_in_ars = Eigen::Quaterniond::Identity();

  /*! Body angular velocity of the IMU frame in [rad] / [s].
   *
   * \note The full name of the body angular vector would be "angular velocity
   * IMU to world in IMU", but as for all body angular velocities, "angular
   * velocity Body to X in Body" is the same for all inertial frames X (that
   * have zero velocity relative to the world frame). See for instance
   * https://scaron.info/robotics/screw-theory.html#body-screws for the math.
   */
  Eigen::Vector3d angular_velocity_imu_in_imu = Eigen::Vector3d::Zero();

  /*! Body linear acceleration of the IMU, in [m] / [s]².
   *
   * \note This quantity corresponds to SPI register 34 from the pi3hat
   * https://github.com/mjbots/pi3hat/blob/master/docs/reference.md#imu-register-mapping
   * from which gravity has been substracted out. Raw IMU data (including
   * gravity) is returned in register 33.
   */
  Eigen::Vector3d linear_acceleration_imu_in_imu = Eigen::Vector3d::Zero();
};

}  // namespace upkie::actuation
