/*
 * Copyright 2022 Stéphane Caron
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <palimpsest/Dictionary.h>

#include <Eigen/Core>
#include <map>
#include <string>
#include <vector>

#include "vulp/observation/Observer.h"

namespace upkie_locomotion::observers {

using palimpsest::Dictionary;
using vulp::observation::Observer;

/*! Floating base odometry from wheel measurements.
 *
 * We can use a simple rigid body for each wheel. In the base frame:
 *
 *    [ v_i.x ]     [ v.x ]     [   0   ]     [ r_i.x ]
 *    [ v_i.y ] =   [ v.y ] +   [   0   ] x   [ r_i.y ] + B_dot(r_i)
 *  B_[ v_i.z ]   B_[  0  ]   B_[ omega ]   B_[   0   ]
 *
 * where v_i is the linear velocity of the wheel point, v the linear velocity
 * of the base, omega its angular velocity, r_i the position of the wheel point
 * and dot(r_i) its time derivative (e.g. if the leg is moving).
 *
 * When we infer all these equations should be satisfied simultaneously. The
 * system is over-determined so we can do it in a least-squares sense.
 *
 * In this implementation, we assume omega and dot(r_i) are zero, so that it
 * all simplifies to:
 *
 *    [ v_i.x ] =   [ v.x ]
 *  B_[ v_i.y ]   B_[ v.y ]
 *
 * And we simply compute v from v_i's with an arithmetic average.
 *
 * \note Odometry is only incremented when at least one wheel is in contact
 * with the ground.
 */
class WheelOdometry : public Observer {
 public:
  //! Observer parameters.
  struct Parameters {
    /*! Configure from dictionary.
     *
     * \param[in] config Global configuration dictionary.
     */
    void configure(const Dictionary& config) {
      if (config.has("wheel_odometry") &&
          config("wheel_odometry").has("signed_radius")) {
        const auto& signed_radius_dict =
            config("wheel_odometry")("signed_radius");
        signed_radius.clear();
        for (const auto& key : signed_radius_dict.keys()) {
          signed_radius[key] = signed_radius_dict(key);
        }
      }
    }

    //! Spine timestep in [s]
    double dt;

    //! Signed radius in [m] for each wheel
    std::map<std::string, double> signed_radius;
  };

  /*! Initialize observer.
   *
   * \param[in] params Observer parameters.
   */
  explicit WheelOdometry(const Parameters& params);

  /*! Observe leg contacts.
   *
   * \param[in] observation Dictionary to read other observations from.
   */
  void read(const Dictionary& observation) final;

  //! Prefix of outputs in the observation dictionary.
  inline std::string prefix() const noexcept final { return "wheel_odometry"; }

  /*! Reset observer.
   *
   * \param[in] config Global configuration dictionary.
   */
  void reset(const Dictionary& config) override;

  /*! Write outputs, called if reading was successful.
   *
   * \param[out] observation Dictionary to write observations to.
   */
  void write(Dictionary& observation) final;

 private:
  /*! Compute average linear velocity from wheels in contact.
   *
   * \param[in] floor_contact Wheel contact states.
   * \param[in] servo Wheel servo readings.
   *
   * \return Average linear velocity in [m] / [s].
   */
  double compute_average_velocity(const Dictionary& floor_contact,
                                  const Dictionary& servo);

 private:
  //! Observer parameters.
  Parameters params_;

  /*! Sagittal ground position in [m].
   *
   * Its origin is defined the first time the robot hits the ground.
   */
  double position_;

  //! Sagittal ground velocity in [m] / [s].
  double velocity_;
};

}  // namespace upkie_locomotion::observers
