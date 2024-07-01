// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <stdexcept>
#include <string>

#include "upkie/cpp/exceptions/FilterError.h"

namespace upkie::cpp::utils {

using upkie::cpp::exceptions::FilterError;

/*! Low-pass filter as an inline function.
 *
 * \param prev_output Previous filter output, or initial value.
 * \param cutoff_period Cutoff period in [s].
 * \param new_input New filter input.
 * \param dt Sampling period in [s].
 *
 * \return New filter output.
 */
inline double low_pass_filter(double prev_output, double cutoff_period,
                              double new_input, double dt) {
  // Make sure the cutoff period is not too small
  if (cutoff_period <= 2.0 * dt) {
    auto message =
        std::string("[low_pass_filter] Cutoff period ") +
        std::to_string(cutoff_period) +
        " s is less than 2 * dt = " + std::to_string(2.0 * dt) +
        " s, causing information loss (Nyquist–Shannon sampling theorem)";
    throw FilterError(message);
  }

  // Actual filtering ;)
  const double alpha = dt / cutoff_period;
  return prev_output + alpha * (new_input - prev_output);
}

}  // namespace upkie::cpp::utils
