// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>
#include <spdlog/fmt/ostr.h>

#include <cmath>
#include <string>

#include "upkie/cpp/observers/Observer.h"

namespace upkie::cpp::observers {

using palimpsest::Dictionary;

/*! Get the orientation of the base frame with respect to the world frame.

 * \param quat_imu_in_ars Quaternion representing the rotation matrix from the
 *     IMU frame to the  attitude reference system (ARS) frame, in ``[w, x, y,
 *     z]`` format.
 * \param rotation_base_to_imu Rotation matrix from the base frame to the IMU
 *     frame. When not specified, the default Upkie mounting orientation is
 *     used.
 * \param rotation_ars_to_world Rotation from the attitude reference system
 *     (ARS) frame to the world frame.
 * \return Rotation matrix from the base frame to the world frame.
 */
inline Eigen::Matrix3d compute_base_orientation_from_imu(
    const Eigen::Quaterniond& quat_imu_in_ars,
    const Eigen::Matrix3d& rotation_base_to_imu,
    const Eigen::Matrix3d& rotation_ars_to_world) {
  Eigen::Matrix3d rotation_imu_to_ars = quat_imu_in_ars.toRotationMatrix();
  Eigen::Matrix3d rotation_base_to_world =
      rotation_ars_to_world * rotation_imu_to_ars * rotation_base_to_imu;
  return rotation_base_to_world;
}

/*! Get pitch angle of a given frame relative to the parent vertical.
 *
 * Figure:
 *
 * ```
 *     parent z
 *       ^    frame z
 *       |     /
 *       |    /
 *       |   /
 *       |  /
 *       | /
 *       |/
 *      (x)-----------> heading vector
 *        \\  )
 *         \\+  positive pitch
 *          \\
 *           \\
 *            \\
 *           frame x
 * ```
 *
 * \param orientation_frame_in_parent Rotation matrix from the target frame to
 *     the parent frame.
 *
 * \return Angle from the parent z-axis (gravity) to the frame z-axis.
 *     Equivalently, angle from the heading vector to the sagittal vector of
 *     the frame.
 *
 * \note Angle is positive in the trigonometric sense (CCW positive, CW
 * negative) in the heading-vertical plane directed by the lateral vector,
 * which is ``(x)`` in the above schematic (pointing away) and not ``(.)``
 * (poiting from screen to the reader).
 */
inline double compute_pitch_frame_in_parent(
    const Eigen::Matrix3d& orientation_frame_in_parent) {
  Eigen::Vector3d sagittal = orientation_frame_in_parent.block<3, 1>(0, 0);
  sagittal.normalize();  // needed for precision around cos_pitch=0.
  Eigen::Vector3d heading_in_parent =
      sagittal - sagittal.z() * Eigen::Vector3d{0.0, 0.0, 1.0};
  heading_in_parent.normalize();
  if (orientation_frame_in_parent(2, 2) < 0.0) {
    heading_in_parent *= -1.0;
  }
  double sign = (sagittal.z() < 0.0) ? +1.0 : -1;
  double cos_pitch = sagittal.dot(heading_in_parent);
  if (cos_pitch < -1.0) {
    cos_pitch = -1.0;
  } else if (cos_pitch > 1.0) {
    cos_pitch = 1.0;
  }
  double pitch = sign * acos(cos_pitch);
  return pitch;
}

/*! Get pitch angle of the base frame relative to the world frame.
 *
 * \param[in] quat_imu_in_ars Quaternion representing the rotation matrix from
 *     the IMU frame to the  attitude reference system (ARS) frame.
 * \param[in] rotation_base_to_imu Rotation matrix from the base frame to the
 *     IMU frame. When not specified, the default Upkie mounting orientation is
 *     used.
 * \param rotation_ars_to_world Rotation from the attitude reference system
 *     (ARS) frame to the world frame.
 * \return Angle from the world z-axis (unit vector opposite to gravity) to the
 *     base z-axis. This angle is positive when the base leans forward.
 */
inline double compute_base_pitch_from_imu(
    const Eigen::Quaterniond& quat_imu_in_ars,
    const Eigen::Matrix3d& rotation_base_to_imu,
    const Eigen::Matrix3d& rotation_ars_to_world) {
  Eigen::Matrix3d rotation_base_to_world = compute_base_orientation_from_imu(
      quat_imu_in_ars, rotation_base_to_imu, rotation_ars_to_world);
  double pitch_base_in_world =
      compute_pitch_frame_in_parent(rotation_base_to_world);
  return pitch_base_in_world;
}

/*! Compute the body angular velocity of the base from IMU readings.
 *
 * \param[in] angular_velocity_imu_in_imu Angular velocity from the IMU.
 * \param[in] rotation_base_to_imu Rotation matrix from base to IMU.
 * \return Body angular velocity of the base frame.
 *
 * Calculation checks:
 *
 * - \f$I\f$: IMU frame
 * - \f$B\f$: base frame
 * - \f$W\f$: world (inertial) frame
 *
 * Our input is \f${}_I \omega_{WI}\f$, we seek \f${}_B \omega_{WB}\f$.
 *
 * \f$
 * \begin{align*}
 *     \dot{R}_{WI} & = R_{WI} ({}_I \omega_{WI} \times) \\
 *     R_{WB} & = R_{WI} R_{IB} \\
 *     \dot{R}_{WB} & = \dot{R}_{WI} R_{IB} \\
 *     & = R_{WI} ({}_I \omega_{WI} \times) R_{IB} \\
 *     & = R_{WI} R_{IB} ({}_B \omega_{WI} \times) \\
 *     & = R_{WB} ({}_B \omega_{WI} \times) = R_{WB} ({}_B \omega_{WB} \times)
 * \end{align*}
 * \f$
 *
 * Thus \f${}_B \omega_{WB} = R_{BI} {}_I \omega_{WI}\f$.
 */
inline Eigen::Vector3d compute_base_angular_velocity_from_imu(
    const Eigen::Vector3d& angular_velocity_imu_in_imu,
    const Eigen::Matrix3d& rotation_base_to_imu) {
  return rotation_base_to_imu.transpose() * angular_velocity_imu_in_imu;
}

//! Observe the orientation of the base frame relative to the world frame.
class BaseOrientation : public Observer {
 public:
  //! Parameters for the base orientation observer
  struct Parameters {
    /*! Configure from dictionary.
     *
     * \param[in] config Global configuration dictionary.
     */
    void configure(const Dictionary& config) {
      // Default IMU and ARS orientations
      rotation_base_to_imu.setZero();
      rotation_base_to_imu.diagonal() << -1.0, 1.0, -1.0;
      rotation_ars_to_world.setZero();
      rotation_ars_to_world.diagonal() << 1.0, -1.0, -1.0;

      if (!config.has("base_orientation")) {
        spdlog::debug("No \"base_orientation\" runtime configuration");
        return;
      }

      auto& observer_config = config("base_orientation");
      if (observer_config.has("rotation_base_to_imu")) {
        rotation_base_to_imu =
            observer_config.get<Eigen::Matrix3d>("rotation_base_to_imu");
        spdlog::info("- rotation_base_to_imu = {}", rotation_base_to_imu);
      }
    }

    /*! Rotation matrix from the base frame to the IMU frame
     *
     * Default Upkie mounting orientation: USB connectors of the raspi pointing
     * to the left of the robot (XT-30 of the pi3hat to the right)
     */
    Eigen::Matrix3d rotation_base_to_imu;

    /*! Rotation matrix from the ARS frame to the world frame
     *
     * The attitude reference system frame has +x forward, +y right and +z
     * down, whereas our world frame has +x forward, +y left and +z up:
     * https://github.com/mjbots/pi3hat/blob/ab632c82bd501b9fcb6f8200df0551989292b7a1/docs/reference.md#orientation
     */
    Eigen::Matrix3d rotation_ars_to_world;
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

  //! Pitch angle of the base frame in the world frame
  double pitch_base_in_world_;

  //! Body angular velocity of the base frame in [rad] / [s]
  Eigen::Vector3d angular_velocity_base_in_base_;
};

}  // namespace upkie::cpp::observers
