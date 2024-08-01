// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <Eigen/Geometry>

//! Utility functions used in the pi3hat interface.
namespace upkie::cpp::actuation::pi3hat {

/*! Recompute raw linear acceleration from UKF observations.
 *
 * \param[in] attitude UKF output orientation.
 * \param[in] accel_mps2 UKF output linear acceleration, in [m] / [s]².
 * \return Raw linear acceleration, in [m] / [s]².
 */
inline Eigen::Vector3d get_raw_linear_acceleration(
    const Eigen::Quaterniond& attitude,
    const Eigen::Vector3d& accel_mps2) noexcept {
  // https://github.com/mjbots/pi3hat/blob/4a3158c831da125fa9c96d64378515c1fdb2083f/fw/attitude_reference.h#L186-L187
  Eigen::Vector3d downward_in_world = {0., 0., -1.};
  Eigen::Vector3d downward_in_imu = attitude.conjugate() * downward_in_world;

  // https://github.com/mjbots/pi3hat/blob/4a3158c831da125fa9c96d64378515c1fdb2083f/fw/attitude_reference.h#L147
  Eigen::Vector3d current_accel_mps2_ = accel_mps2 + 9.81 * downward_in_imu;
  return current_accel_mps2_;
}

}  // namespace upkie::cpp::actuation::pi3hat
