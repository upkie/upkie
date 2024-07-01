// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>

//! WIP: Process higher-level actions down to joint commands.
namespace upkie {

using palimpsest::Dictionary;

//! Base class for controllers.
class Controller {
 public:
  /*! Decide action.
   *
   * \param[out] action Dictionary to write the full action to.
   * \param[in] observation Latest observation.
   */
  virtual void act(Dictionary& action, const Dictionary& observation) = 0;
};

}  // namespace upkie
