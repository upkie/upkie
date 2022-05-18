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

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "robots/upkie/observers/WheelContact.h"
#include "vulp/observation/Observer.h"

namespace robots::upkie::observers {

using palimpsest::Dictionary;
using vulp::observation::Observer;

//! Observer for leg contacts.
class FloorContact : public Observer {
 public:
  struct Parameters {
    /*! Initialize from global configuration.
     *
     * \param[in] config Global configuration dictionary.
     */
    explicit Parameters(const Dictionary& config)
        : wheel_contact_params(config) {
      // TODO(scaron): remove config constructor argument.
      configure(config);
    }

    /*! Configure from dictionary.
     *
     * \param[in] config Global configuration dictionary.
     */
    void configure(const Dictionary& config) {
      if (config.has("spine_frequency")) {
        dt = 1.0 / config.get<unsigned>("spine_frequency");
      }
      if (config.has("floor_contact")) {
        upper_leg_torque_threshold =
            config("floor_contact")("upper_leg_torque_threshold");
      }
      wheel_contact_params.configure(config);
    }

    //! Spine timestep, in [s]
    double dt = std::numeric_limits<double>::quiet_NaN();

    //! List of upper leg (hip, knee, ...) joints
    std::vector<std::string> upper_leg_joints;

    //! List of wheel joint names
    std::vector<std::string> wheels;

    //! Wheel contact observer configuration
    WheelContact::Parameters wheel_contact_params;

    //! Threshold for upper leg torques, in [N.m]
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

  //! Norm of the upper-leg joint torque vector, in [N.m]
  double upper_leg_torque_;

  //! True if and only if the observer detects a contact.
  bool contact_;
};

}  // namespace robots::upkie::observers
