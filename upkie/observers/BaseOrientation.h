// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>

#include "vulp/observation/Observer.h"

namespace upkie::observers {

/*! Get pitch angle of the base frame relative to the world frame.
 *
 * \param[in] quat_imu_in_ars Quaternion representing the rotation matrix from
 *     the IMU frame to the  attitude reference system (ARS) frame.
 * \param[in] rotation_base_to_imu Rotation matrix from the base frame to the
 *     IMU frame. When not specified, the default Upkie mounting orientation is
 *     used.
 * \return Angle from the world z-axis (unit vector opposite to gravity) to the
 *     base z-axis. This angle is positive when the base leans forward.
 */
double compute_base_pitch_from_imu(Eigen::Quaterniond quat_imu_in_ars,
                                   Eigen::Matrix3d rotation_base_to_imu) {
  rotation_base_to_world =
      compute_base_orientation_from_imu(quat_imu_in_ars, rotation_base_to_imu);
  pitch_base_in_world = compute_pitch_frame_in_parent(rotation_base_to_world);
  return pitch_base_in_world;
}

/*!
 */
class BaseOrientation : public Observer {
 public:
  struct Parameters {
    /*! Configure from dictionary.
     *
     * \param[in] config Global configuration dictionary.
     */
    void configure(const Dictionary& config) {
      // Default rotation from base to IMU
      rotation_base_to_imu.setZero();
      rotation_base_to_imu.diagonal() << -1.0, 1.0, -1.0;

      if (!config.has("base_orientation")) {
        spdlog::debug("No \"base_orientation\" runtime configuration");
        return;
      }
    }

    Eigen::Matrix3d rotation_base_to_imu;
  };

  /*! Initialize observer.
   *
   * \param[in] params Observer parameters.
   */
  explicit BaseOrientation(const Parameters& params);

  //! Prefix of outputs in the observation dictionary.
  inline std::string prefix() const noexcept final {
    return "base_orientation";
  }

  /*! Reset observer.
   *
   * \param[in] config Overall configuration dictionary.
   */
  void reset(const Dictionary& config) override;

  /*! Read inputs from other observations.
   *
   * \param[in] observation Dictionary to read other observations from.
   */
  void read(const Dictionary& observation) final;

  /*! Write outputs, called if reading was successful.
   *
   * \param[out] observation Dictionary to write observations to.
   */
  void write(Dictionary& observation) final;

 private:
  //! Observer parameters
  Parameters params_;
}

