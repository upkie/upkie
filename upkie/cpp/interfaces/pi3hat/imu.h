// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Stéphane Caron

#pragma once

#include <mjbots/pi3hat/pi3hat.h>

#include <Eigen/Geometry>

#include "upkie/cpp/interfaces/pi3hat/imu_eigen.h"

//! Utility functions used in the pi3hat interface.
namespace upkie::cpp::interfaces::pi3hat {

//! Attitude data type from the mjbots library.
using Attitude = ::mjbots::pi3hat::Attitude;

/*! Get orientation from the IMU frame to the ARS frame.
 *
 * \param[in] attitude Attitude object from the pi3hat library.
 * \return Orientation from the IMU frame to the ARS frame.
 *
 * This orientation is computed by the Unscented Kalman filter (UKF) in
 * pi3hat/fw/attitude_reference.h.
 */
inline Eigen::Quaterniond get_orientation_imu_in_ars(
    const Attitude& attitude) noexcept {
  const double w = attitude.attitude.w;
  const double x = attitude.attitude.x;
  const double y = attitude.attitude.y;
  const double z = attitude.attitude.z;
  // These values were floats so the resulting quaternion is only
  // approximately normalized. We saw this property in d7fcaa97fa.
  return Eigen::Quaterniond(w, x, y, z).normalized();
}

/*! Get the body angular velocity of the IMU frame.
 *
 * \param[in] attitude Attitude object from the pi3hat library.
 * \return Body angular velocity of the IMU frame in [rad] / [s].
 *
 * \note This is the angular velocity \f$ {}_I \omega_{WI} \f$ from the IMU
 * frame \f$ I \f$ to the world frame \f$ W \f$, expressed in the IMU frame.
 */
inline Eigen::Vector3d get_angular_velocity(const Attitude& attitude) noexcept {
  const double omega_x = attitude.rate_dps.x * M_PI / 180.;
  const double omega_y = attitude.rate_dps.y * M_PI / 180.;
  const double omega_z = attitude.rate_dps.z * M_PI / 180.;
  return {omega_x, omega_y, omega_z};
}

/*! Get the body linear acceleration of the IMU.
 *
 * \param[in] attitude Attitude object from the pi3hat library.
 * \return Body linear acceleration of the IMU in [m] / [s]².
 *
 * \note This is the linear acceleration \f$ {}_I a_{WI} \f$ of the IMU frame
 * \f$ I \f$ with respect to the world frame, expressed in the IMU frame.
 */
inline Eigen::Vector3d get_linear_acceleration(
    const Attitude& attitude) noexcept {
  const double a_x = attitude.accel_mps2.x;
  const double a_y = attitude.accel_mps2.y;
  const double a_z = attitude.accel_mps2.z;
  return {a_x, a_y, a_z};
}

/*! Get the body angular velocity of the IMU frame in [deg] / [s].
 *
 * \param[in] attitude Attitude object from the pi3hat library.
 * \return Body angular velocity of the IMU frame in [deg] / [s].
 */
inline Eigen::Vector3d get_rate_dps(const Attitude& attitude) noexcept {
  const double omega_x = attitude.rate_dps.x;
  const double omega_y = attitude.rate_dps.y;
  const double omega_z = attitude.rate_dps.z;
  return {omega_x, omega_y, omega_z};
}

/*! Get the gyroscope bias in [deg] / [s].
 *
 * \param[in] attitude Attitude object from the pi3hat library.
 * \return Gyroscope bias of the IMU in [deg] / [s].
 */
inline Eigen::Vector3d get_bias_dps(const Attitude& attitude) noexcept {
  const double b_x = attitude.bias_dps.x;
  const double b_y = attitude.bias_dps.y;
  const double b_z = attitude.bias_dps.z;
  return {b_x, b_y, b_z};
}

}  // namespace upkie::cpp::interfaces::pi3hat
