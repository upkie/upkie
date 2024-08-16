// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Inria

#pragma once

#include <palimpsest/Dictionary.h>

#include <Eigen/Core>

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"

namespace upkie::cpp::actuation {

using palimpsest::Dictionary;

//! Uncertainty on IMU measurements.
struct ImuUncertainty {
  //! Bias vector added to accelerometer measurements, in the IMU frame.
  Eigen::Vector3d accelerometer_bias = Eigen::Vector3d::Zero();

  //! Standard deviations of accelerometer measurement noise, in the IMU frame.
  Eigen::Vector3d accelerometer_noise = Eigen::Vector3d::Zero();

  //! Bias vector added to gyroscope measurements, in the IMU frame.
  Eigen::Vector3d gyroscope_bias = Eigen::Vector3d::Zero();

  //! Standard deviations of gyroscope measurement noise, in the IMU frame.
  Eigen::Vector3d gyroscope_noise = Eigen::Vector3d::Zero();

  //! Default constructor for uncertainty initialized to default values.
  ImuUncertainty() = default;

  /*! Initialize configurable properties from a dictionary.
   *
   * \param[in] config Configuration dictionary.
   */
  explicit ImuUncertainty(const Dictionary& config) {
    accelerometer_bias = config.get<Eigen::Vector3d>("accelerometer_bias",
                                                     Eigen::Vector3d::Zero());
    accelerometer_noise = config.get<Eigen::Vector3d>("accelerometer_noise",
                                                      Eigen::Vector3d::Zero());
    gyroscope_bias =
        config.get<Eigen::Vector3d>("gyroscope_bias", Eigen::Vector3d::Zero());
    gyroscope_noise =
        config.get<Eigen::Vector3d>("gyroscope_noise", Eigen::Vector3d::Zero());
  }
};

}  // namespace upkie::cpp::actuation
