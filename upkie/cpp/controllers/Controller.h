// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>

#include <string>

namespace upkie::cpp::controllers {

using palimpsest::Dictionary;

/*! Base class for controllers.
 *
 * Controllers read from the action and observation dictionary, and write to
 * the action dictionary.
 */
class Controller {
 public:
  //! Destructor is virtual to deallocate lists of controllers properly.
  virtual ~Controller() {}

  //! Name of the controller.
  virtual inline std::string name() const noexcept {
    return "unknown_controller";
  }

  /*! Reset controller.
   *
   * \param[in] config Configuration dictionary.
   */
  virtual void reset(const Dictionary& config) {}

  /*! Read inputs from the observation and action dictionaries.
   *
   * \param[in] observation Dictionary to read other observations from.
   *
   * \note The base class reads nothing. We put an empty function here rather
   * than making the class abstract to be able to instantiate vectors of it.
   */
  virtual void read(const Dictionary& observation, const Dictionary& action) {}

  /*! Write outputs, only called if reading was successful.
   *
   * \param[out] action Dictionary to write actions to.
   *
   * \note The base class writes nothing. We put an empty function here rather
   * than making the class abstract to be able to instantiate vectors of it.
   */
  virtual void write(Dictionary& action) {}
};

}  // namespace upkie::cpp::controllers
