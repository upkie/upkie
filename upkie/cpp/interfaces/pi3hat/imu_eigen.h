// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria
//
// Tests in this file only depend on Eigen, so that we can execute them on x86.

#pragma once

#include <Eigen/Geometry>

namespace upkie::cpp::interfaces::pi3hat {

/*! Standard gravity constant used in the mjbots pi3hat firmware.
 *
 * We should use the same value to recover raw measurements from filter
 * outputs.
 */
constexpr double kMjbotsGravity = 9.81;

/*! Get raw angular velocity measurement from the gyroscope.
 *
 * \param[in] rate_dps Angular velocity in deg/s from the pi3hat.
 * \param[in] bias_dps Gyroscope bias in deg/s from the pi3hat.
 * \return Raw angular velocity of the IMU in rad/s.
 *
 * \note We reverse filter outputs for now as (1) it avoids an extra call to
 * ReadSpi, and more generally customizing the Pi3Hat::Impl from mjbots, and
 * (2) the pi3hat filter's code has been stable for the last four years.
 */
inline Eigen::Vector3d get_raw_angular_velocity(
    const Eigen::Vector3d& rate_dps, const Eigen::Vector3d& bias_dps) noexcept {
  // https://github.com/mjbots/pi3hat/blob/4a3158c831da125fa9c96d64378515c1fdb2083f/fw/attitude_reference.h#L143
  Eigen::Vector3d current_gyro_dps = rate_dps - bias_dps;
  return current_gyro_dps * M_PI / 180.;
}

/*! Recompute raw linear acceleration from UKF observations.
 *
 * \param[in] orientation_imu_in_ars Rotation from the IMU to the ARS.
 * \param[in] accel_mps2 UKF output linear acceleration, in m/s².
 * \return Raw linear acceleration in m/s².
 *
 * \note We reverse filter outputs for now as (1) it avoids an extra call to
 * ReadSpi, and more generally customizing the Pi3Hat::Impl from mjbots, and
 * (2) the pi3hat filter's code has been stable for the last four years.
 */
inline Eigen::Vector3d get_raw_linear_acceleration(
    const Eigen::Quaterniond& orientation_imu_in_ars,
    const Eigen::Vector3d& accel_mps2) noexcept {
  // https://github.com/mjbots/pi3hat/blob/4a3158c831da125fa9c96d64378515c1fdb2083f/fw/attitude_reference.h#L186-L187
  Eigen::Vector3d u_in_ars = {0., 0., -1.};  // downward in ARS, upward in world
  Eigen::Vector3d u_in_imu = orientation_imu_in_ars.conjugate() * u_in_ars;

  // https://github.com/mjbots/pi3hat/blob/4a3158c831da125fa9c96d64378515c1fdb2083f/fw/attitude_reference.h#L147
  Eigen::Vector3d current_accel_mps2 = accel_mps2 + kMjbotsGravity * u_in_imu;
  return current_accel_mps2;
}

}  // namespace upkie::cpp::interfaces::pi3hat
