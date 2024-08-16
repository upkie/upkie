// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <palimpsest/Dictionary.h>

#include <random>

namespace upkie::cpp::actuation {

using palimpsest::Dictionary;

inline void add_vector_uncertainty(Eigen::Vector3d& output, double bias,
                                   double noise, std::mt19937& rng) {
  std::normal_distribution<double> uncertainty(bias, noise);
  output.x() += uncertainty(rng);
  output.y() += uncertainty(rng);
  output.z() += uncertainty(rng);
}

//! Uncertainty on IMU measurements.
struct ImuUncertainty {
  //! Bias added to accelerometer measurements in the IMU frame.
  double accelerometer_bias = 0.0;

  //! Standard deviation of noise added to accelerations in the IMU frame.
  double accelerometer_noise = 0.0;

  //! Bias added to gyroscope measurements, in the IMU frame.
  double gyroscope_bias = 0.0;

  //! Standard deviation of noise added to angular velocities in the IMU frame.
  double gyroscope_noise = 0.0;

  /*! Configure uncertainty from a dictionary.
   *
   * \param[in] config Configuration dictionary.
   */
  void configure(const Dictionary& config) {
    accelerometer_bias = config.get<double>("accelerometer_bias", 0.0);
    accelerometer_noise = config.get<double>("accelerometer_noise", 0.0);
    gyroscope_bias = config.get<double>("gyroscope_bias", 0.0);
    gyroscope_noise = config.get<double>("gyroscope_noise", 0.0);
  }

  /*! Apply uncertainty to measurement vectors.
   *
   * \param[in,out] linear_acceleration Linear acceleration measurement, in the
   *     IMU frame.
   * \param[in,out] angular_velocity Angular velocity measurement, in the IMU
   *     frame.
   */
  void apply(Eigen::Vector3d& linear_acceleration,
             Eigen::Vector3d& angular_velocity, std::mt19937& rng) const {
    add_vector_uncertainty(linear_acceleration, accelerometer_bias,
                           accelerometer_noise, rng);
    add_vector_uncertainty(angular_velocity, gyroscope_bias, gyroscope_noise,
                           rng);
  }
};

}  // namespace upkie::cpp::actuation
