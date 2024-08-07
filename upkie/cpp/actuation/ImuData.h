// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace upkie::cpp::actuation {

//! Data from an onboard IMU such as the pi3hat's.
struct ImuData {
  /*! Rotation from the IMU frame to the ARS frame.
   *
   * The attitude reference system frame has +x forward, +y right and +z down,
   * whereas our world frame has +x forward, +y left and +z up:
   * https://github.com/mjbots/pi3hat/blob/ab632c82bd501b9fcb6f8200df0551989292b7a1/docs/reference.md#orientation
   */
  Eigen::Quaterniond orientation_imu_in_ars = Eigen::Quaterniond::Identity();

  /*! Body angular velocity of the IMU frame in [rad] / [s].
   *
   * The full name of the body angular vector would be "angular velocity IMU to
   * world in IMU", but as for all body angular velocities, "angular velocity
   * Body to X in Body" is the same for all inertial frames X (that have zero
   * velocity relative to the world frame). See for instance
   * https://scaron.info/robotics/screw-theory.html#body-screws for the math.
   */
  Eigen::Vector3d angular_velocity_imu_in_imu = Eigen::Vector3d::Zero();

  /*! Linear acceleration of the IMU, with gravity filtered out, in [m] / [s]².
   *
   * This quantity corresponds to SPI register 34 from the pi3hat
   * https://github.com/mjbots/pi3hat/blob/master/docs/reference.md#imu-register-mapping
   * from which gravity has been substracted out
   * https://github.com/mjbots/pi3hat/blob/4a3158c831da125fa9c96d64378515c1fdb2083f/fw/attitude_reference.h#L147.
   */
  Eigen::Vector3d linear_acceleration_imu_in_imu = Eigen::Vector3d::Zero();

  /*! Raw angular velocity read by the IMU, in [rad] / [s].
   *
   * \note This raw angular velocity measurement has more bias than the
   * filtered one.
   */
  Eigen::Vector3d raw_angular_velocity = Eigen::Vector3d::Zero();

  /*! Proper linear acceleration read by the IMU, in [m] / [s]².
   *
   * The acceleration read by an accelerometer is:
   * \f[
   * {}_I a_{WI} - {}_I g
   * \f]
   * with \f${}_I a_{WI}\f$ the *proper* linear acceleration of the IMU frame
   * \f$I\f$, and \f${}_I g\f$ the standard acceleration of gravity in the IMU
   * frame. Pay attention to the fact that
   *
   * \note Pay attention to the fact that proper acceleration is different from
   * coordinate acceleration, which would be the acceleration of the IMU with
   * respect to the mobile IMU frame in this case. (There is a gyroscopic term
   * in the latter but not in the former.) See for instance the Wikipedia
   * [Accelerometer](https://en.wikipedia.org/wiki/Accelerometer) article.
   */
  Eigen::Vector3d raw_linear_acceleration = Eigen::Vector3d::Zero();
};

}  // namespace upkie::cpp::actuation
