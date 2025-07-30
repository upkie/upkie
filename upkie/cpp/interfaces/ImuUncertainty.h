// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <palimpsest/Dictionary.h>

#include <Eigen/Core>
#include <random>

namespace upkie::cpp::interfaces {

using palimpsest::Dictionary;

/*! Add white noise to a vector.
 *
 * \param[out] vector Vector to add noise to.
 * \param[in] std_dev Standard deviation of white noise.
 * \param[in,out] rng Random number generator.
 */
inline void add_white_noise(Eigen::Vector3d& vector, double std_dev,
                            std::mt19937& rng) {
  std::normal_distribution<double> uncertainty(0.0, std_dev);
  vector.x() += uncertainty(rng);
  vector.y() += uncertainty(rng);
  vector.z() += uncertainty(rng);
}

//! Uncertainty on IMU measurements.
struct ImuUncertainty {
  //! Bias added to accelerometer measurements in the IMU frame.
  Eigen::Vector3d accelerometer_bias = Eigen::Vector3d::Zero();

  //! Standard deviation of noise added to accelerations in the IMU frame.
  double accelerometer_noise = 0.0;

  //! Bias added to gyroscope measurements, in the IMU frame.
  Eigen::Vector3d gyroscope_bias = Eigen::Vector3d::Zero();

  //! Standard deviation of noise added to angular velocities in the IMU frame.
  double gyroscope_noise = 0.0;

  /*! Configure uncertainty from a dictionary.
   *
   * \param[in] config Configuration dictionary.
   */
  void configure(const Dictionary& config) {
    accelerometer_bias = config.get<Eigen::Vector3d>("accelerometer_bias",
                                                     Eigen::Vector3d::Zero());
    accelerometer_noise = config.get<double>("accelerometer_noise", 0.0);
    gyroscope_bias =
        config.get<Eigen::Vector3d>("gyroscope_bias", Eigen::Vector3d::Zero());
    gyroscope_noise = config.get<double>("gyroscope_noise", 0.0);
  }

  /*! Apply uncertainty to measurement vectors.
   *
   * \param[in,out] linear_acceleration Linear acceleration measurement, in the
   *     IMU frame.
   * \param[in,out] angular_velocity Angular velocity measurement, in the IMU
   *     frame.
   * \param[in,out] rng Random number generator.
   */
  void apply(Eigen::Vector3d& linear_acceleration,
             Eigen::Vector3d& angular_velocity, std::mt19937& rng) const {
    linear_acceleration += accelerometer_bias;
    angular_velocity += gyroscope_bias;
    add_white_noise(linear_acceleration, accelerometer_noise, rng);
    add_white_noise(angular_velocity, gyroscope_noise, rng);
  }
};

}  // namespace upkie::cpp::interfaces
