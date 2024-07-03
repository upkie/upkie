// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>

#include <string>

namespace upkie::observers {

using palimpsest::Dictionary;

//! Base class for observers.
class Observer {
 public:
  //! Destructor is virtual to deallocate lists of observers properly.
  virtual ~Observer() {}

  //! Prefix of outputs in the observation dictionary.
  virtual inline std::string prefix() const noexcept {
    return "unknown_source";
  }

  /*! Reset observer.
   *
   * \param[in] config Configuration dictionary.
   */
  virtual void reset(const Dictionary& config) {}

  /*! Read inputs from other observations.
   *
   * \param[in] observation Dictionary to read other observations from.
   *
   * \note The base class reads nothing. We put an empty function here rather
   * than making the class abstract to be able to instantiate vectors of it.
   */
  virtual void read(const Dictionary& observation) {}

  /*! Write outputs, called if reading was successful.
   *
   * \param[out] observation Dictionary to write observations to.
   *
   * \note The base class writes nothing. We put an empty function here rather
   * than making the class abstract to be able to instantiate vectors of it.
   */
  virtual void write(Dictionary& observation) {}
};

}  // namespace upkie::observers
