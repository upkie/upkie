// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "upkie/cpp/observers/Observer.h"
#include "upkie/cpp/observers/WheelContact.h"

/*! State observers available for Upkies.
 *
 * \image html observers.png
 * \image latex observers.eps
 */
namespace upkie::cpp::observers {

using palimpsest::Dictionary;

/*! Observe contact between the wheels and the floor.
 *
 * This observer makes two assumptions:
 *
 * 1. The floor is a plane.
 * 2. The robot's head plane is parallel to the floor.
 *
 * The two planes are depicted in this overview figure:
 *
 * \image html floor-contact.png
 * \image latex floor-contact.eps
 *
 * It determines whether contact is established based on:
 *
 * - WheelContact observers associated with each wheel.
 * - Hip and knee torques, whose norm is a good indicator of contact when the
 *   floor is not orthogonal to gravity, or the legs are bent, but is an
 *   unreliable indicator when the floor is horizontal and the legs are
 *   stretched.
 *
 * The latter is an incremental improvement to filter out outliers from the
 * former.
 *
 */
class FloorContact : public Observer {
 public:
  //! Parameters to the floor contact observer
  struct Parameters {
    /*! Configure from dictionary.
     *
     * \param[in] config Global configuration dictionary.
     */
    void configure(const Dictionary& config) {
      wheel_contact_params.configure(config);

      if (!config.has("floor_contact")) {
        spdlog::debug("No \"floor_contact\" runtime configuration");
        return;
      }

      spdlog::info("Applying \"floor_contact\" runtime configuration");
      upper_leg_torque_threshold =
          config("floor_contact")("upper_leg_torque_threshold");
    }

    /*! Check whether parameters are incomplete.
     *
     * \return True if some parameters are uninitialized.
     */
    bool incomplete() { return (dt != dt); }

    //! Spine timestep, in [s]
    double dt = std::numeric_limits<double>::quiet_NaN();

    //! Wheel contact observer configuration
    WheelContact::Parameters wheel_contact_params;

    //! Threshold for upper leg torques, in N⋅m
    double upper_leg_torque_threshold = 10.0;
  };

  /*! Initialize observer.
   *
   * \param[in] params Observer parameters.
   */
  explicit FloorContact(const Parameters& params);

  //! Prefix of outputs in the observation dictionary.
  inline std::string prefix() const noexcept final { return "floor_contact"; }

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
  /*! Update wheel observers and check whether a wheel is in contact.
   *
   * \param[in] observation Observation.
   * \return True if at least one wheel is in contact.
   */
  bool check_wheel_contacts(const Dictionary& observation);

  //! Update upper leg torque from observation
  void update_upper_leg_torque(const Dictionary& observation);

 private:
  //! Observer parameters
  Parameters params_;

  //! Wheel contact observers
  std::unordered_map<std::string, WheelContact> wheel_contacts_;

  //! Norm of the upper-leg joint torque vector, in N⋅m.
  double upper_leg_torque_;

  //! True if and only if the observer detects a contact.
  bool contact_;

  //! List of hip and knee joint names.
  const std::vector<std::string> upper_leg_joints_;

  //! List of wheel joint names.
  const std::vector<std::string> wheel_joints_;
};

}  // namespace upkie::cpp::observers
