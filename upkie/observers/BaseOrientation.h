// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <palimpsest/Dictionary.h>

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "upkie/observers/WheelContact.h"
#include "vulp/observation/Observer.h"

namespace upkie::observers {

/*!
 */
class BaseOrientation : public Observer {
 public:
  struct Parameters {
    /*! Configure from dictionary.
     *
     * \param[in] config Global configuration dictionary.
     */
    void configure(const Dictionary& config) {
      if (!config.has("base_orientation")) {
        spdlog::debug("No \"base_orientation\" runtime configuration");
        return;
      }
    }
  };

  /*! Initialize observer.
   *
   * \param[in] params Observer parameters.
   */
  explicit BaseOrientation(const Parameters& params);

  //! Prefix of outputs in the observation dictionary.
  inline std::string prefix() const noexcept final {
    return "base_orientation";
  }

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
  //! Observer parameters
  Parameters params_;
}

