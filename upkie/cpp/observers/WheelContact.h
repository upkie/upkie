// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>

#include <Eigen/Core>
#include <algorithm>
#include <limits>

namespace upkie::cpp::observers {

using palimpsest::Dictionary;

/*! Observe contact between a given wheel and the floor.
 *
 * We use an acceleration-from-torque simple linear regressor with no affine
 * term and online updates. There is a quick intro to the math in the following
 * post on [simple linear regression with online
 * updates](https://scaron.info/blog/simple-linear-regression-with-online-updates.html).
 * The output of this regressor is an "inertia" (torque over acceleration
 * slope) that is low during contact and large in the air (as then the wheel
 * needs little torque to spin freely).
 *
 * The observer is made more reliable by three ~~hacks~~ post-processing stages:
 *
 * 1. **Max-passthrough filtering:** We further filter the output inertia by a
 * max-passthrough, to detect contact loss fast, which otherwise decays
 * exponentially, so that we tend to assume contact remains if there is no
 * contact loss for a while.
 *
 * 2. **Touchdown and liftoff hysteresis:** Filtering aside, the other memory
 * in this observer comes from hysteresis. Contact is detected as soon as the
 * contact inertia goes below the configured touchdown inertia, and lost when
 * it spikes above the liftoff inertia. This allows a gray band of variations
 * while the robot moves around in contact.
 *
 * 3. **No decision when acceleration or torque is low:** There is a second
 * hysteresis around low filtered accelerations or low filtered torques. If
 * filtered values are low then real values have been zero for a while, which
 * is inconclusive: the robot could be standing perfectly upright rather than
 * having lost contact. (Most likely, if it loses contact, these values won't
 * stay zero for long anyway.) When this happens we just keep the memorized
 * contact state and wait for more exciting data.
 */
class WheelContact {
 public:
  //! Observer parameters.
  struct Parameters {
    /*! Read parameters from a configuration dictionary.
     *
     * \param[in] config Configuration dictionary.
     */
    void configure(const Dictionary& config) {
      if (!config.has("wheel_contact")) {
        spdlog::debug("No \"wheel_contact\" runtime configuration");
        return;
      }

      spdlog::info("Applying \"wheel_contact\" runtime configuration");
      const Dictionary& wheel_contact = config("wheel_contact");
      liftoff_inertia = wheel_contact.get<double>("liftoff_inertia");
      min_touchdown_acceleration =
          wheel_contact.get<double>("min_touchdown_acceleration");
      min_touchdown_torque = wheel_contact.get<double>("min_touchdown_torque");
      cutoff_period = wheel_contact.get<double>("cutoff_period");
      touchdown_inertia = wheel_contact.get<double>("touchdown_inertia");
    }

    /*! Hysteresis value of the apparent inertia that triggers liftoff
     * detection, in m²·kg.
     */
    double liftoff_inertia;

    //! Low-pass filtering time constant, in seconds.
    double cutoff_period;

    /*! Skip touchdown detection below this acceleration, in rad/s².
     *
     * This avoids false positives when the air wheel acceleration happens to
     * be low.
     */
    double min_touchdown_acceleration;

    //! Skip touchdown detection below this torque, in N⋅m.
    double min_touchdown_torque;

    /*! Hysteresis value of the apparent inertia that triggers touchdown
     * detection, in m²·kg.
     */
    double touchdown_inertia;
  };

  /*! Initialize observer.
   *
   * \param[in] params Observer parameters.
   */
  explicit WheelContact(const Parameters& params);

  /*! Process a new observation.
   *
   * \param[in] torque New torque observation.
   * \param[in] velocity New velocity observation.
   * \param[in] dt Duration in seconds since last observation.
   */
  void observe(const double torque, const double velocity,
               const double dt) noexcept;

  //! Low-pass filtered absolute wheel acceleration, in rad/s²
  double abs_acceleration() const noexcept { return abs_acceleration_; }

  //! Low-pass filtered absolute wheel torque, in N⋅m
  double abs_torque() const noexcept { return abs_torque_; }

  //! Current contact state
  bool contact() const noexcept { return contact_; }

  //! Apparent inertia I = |torque| / |acceleration| at the wheel
  double inertia() const noexcept { return inertia_; }

  //! Reset contact to false
  void reset_contact() noexcept { contact_ = false; }

 private:
  //! Observer parameters.
  const Parameters params_;

  //! Low-pass filtered absolute wheel acceleration, in rad/s²
  double abs_acceleration_;

  //! Low-pass filtered absolute wheel torque, in N⋅m
  double abs_torque_;

  //! Observer contact state
  bool contact_;

  //! Apparent inertia I = |torque| / |acceleration| at the wheel
  double inertia_;

  //! Low-pass filtered wheel velocity, in rad/s
  double velocity_;
};

}  // namespace upkie::cpp::observers
