// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>

#include <string>

//! Sensors (pure outputs) available for Upkies.
namespace upkie::cpp::sensors {

using palimpsest::Dictionary;

/*! Base class for sensors.
 *
 * Sensors run before observers. They write their observations without reading
 * other observations from the pipeline, although they may read data from other
 * media (such as the file system).
 */
class Sensor {
 public:
  //! Destructor is virtual to deallocate lists of observers properly.
  virtual ~Sensor() {}

  /*! Prefix of output in the observation dictionary.
   *
   * This prefix is looked up by the spine to feed `observation(prefix)` to
   * the \ref write function.
   */
  virtual inline std::string prefix() const noexcept {
    return "unknown_sensor";
  }

  /*! Write output to a dictionary.
   *
   * \param[out] observation Dictionary to write observations to.
   *
   * \note The base class writes nothing. We put an empty function here rather
   * than making the class abstract to be able to instantiate vectors of it.
   */
  virtual void write(Dictionary& observation) {}
};

}  // namespace upkie::cpp::sensors
