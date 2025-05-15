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

//! Reset wheel commands to no position and zero velocity.
class WheelStopper : public Controller {
 public:
  /*! Initialize controller.
   *
   * \param[in] params Controller parameters.
   */
  explicit WheelStopper();

  //! Name of the controller.
  inline std::string name() const noexcept final {
    return "reset_wheel_velocities";
  }

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
};

}  // namespace upkie::cpp::controllers
