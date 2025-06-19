// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>

#include <Eigen/Core>
#include <map>
#include <string>
#include <vector>

#include "upkie/cpp/controllers/Controller.h"

namespace upkie::cpp::controllers {

using palimpsest::Dictionary;

//! Balance by feedback control over the base pitch angle and ground position.
class WheelBalancer : public Controller {
 public:
  //! Wheel balancer parameters.
  struct Parameters {
    /*! Configure from dictionary.
     *
     * \param[in] config Global configuration dictionary.
     */
    void configure(const Dictionary& global_config) {
      if (!global_config.has("wheel_balancer")) {
        spdlog::debug("No \"wheel_balancer\" runtime configuration");
        return;
      }
      spdlog::info("Applying \"wheel_balancer\" runtime configuration");
      const auto& config = global_config("wheel_balancer");

      contact_radius = config.get<double>("contact_radius", contact_radius);
      fall_pitch = config.get<double>("fall_pitch", fall_pitch);
      max_ground_velocity =
          config.get("max_ground_velocity", max_ground_velocity);
      pitch_damping = config.get("pitch_damping", pitch_damping);
      pitch_stiffness = config.get("pitch_stiffness", pitch_stiffness);
      position_damping = config.get("position_damping", position_damping);
      position_stiffness = config.get("position_stiffness", position_stiffness);
      stiff_yaw_velocity = config.get("stiff_yaw_velocity", stiff_yaw_velocity);
      wheel_radius = config.get("wheel_radius", wheel_radius);
    }

    //! Half the distance between the two wheels, in meters.
    double contact_radius = 0.1524;

    //! Spine timestep in [s].
    double dt;

    //! Fall pitch angle in [rad].
    double fall_pitch = 1.0;

    //! Maximum ground velocity, in [m] / [s].
    double max_ground_velocity = 2.0;

    /*! Pitch error (normalized) damping gain.
     *
     * Corresponds to the proportional term of the velocity PI controller,
     * equivalent to the derivative term of the acceleration PD controller.
     */
    double pitch_damping = 1.8;

    /*! Pitch error (normalized) stiffness gain.
     *
     * Corresponds to the integral term of the velocity PI controller,
     * equivalent to the proportional term of the acceleration PD controller.
     */
    double pitch_stiffness = 20.0;

    /*! Position error (normalized) damping gain.
     *
     * Corresponds to the proportional term of the velocity PI controller,
     * equivalent to the derivative term of the acceleration PD controller.
     */
    double position_damping = 0.7;

    /*! Position error (normalized) stiffness gain.
     *
     * Corresponds to the integral term of the velocity PI controller,
     * equivalent to the proportional term of the acceleration PD controller.
     */
    double position_stiffness = 1.6;

    //! Magnitude of yaw velocity in [rad] / [s] above which legs get stiffer.
    double stiff_yaw_velocity = 0.1;

    //! Wheel radius in [m].
    double wheel_radius = 0.06;
  };

  /*! Initialize controller.
   *
   * \param[in] params Controller parameters.
   */
  explicit WheelBalancer(const Parameters& params);

  //! Name of the controller.
  inline std::string name() const noexcept final { return "wheel_balancer"; }

  /*! Reset controller.
   *
   * \param[in] config Global configuration dictionary.
   */
  void reset(const Dictionary& config) override;

  /*! Read inputs from the observation and action dictionaries.
   *
   * \param[in] observation Dictionary to read other observations from.
   * \param[in] action Dictionary to read current actions from.
   */
  void read(const Dictionary& observation, const Dictionary& action) final;

  /*! Write outputs, only called if reading was successful.
   *
   * \param[out] action Dictionary to write actions to.
   */
  void write(Dictionary& observation) final;

 private:
  //! Controller parameters.
  Parameters params_;

  //! Ground velocity offset used for balancing, in [m] / [s].
  double ground_velocity_;

  //! Ground velocity offset due to the integral error, in [m] / [s].
  double integral_velocity_;

  //! Target ground position in [m].
  double target_ground_position_;

  //! Target yaw velocity in [rad] / [s].
  double target_yaw_velocity_;
};

}  // namespace upkie::cpp::controllers
