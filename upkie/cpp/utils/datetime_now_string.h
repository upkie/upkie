// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

namespace upkie::utils {

/*! Generate a date-time string.
 *
 * \return Date-time string at the time the function is called.
 *
 * Uses the same format as the Python utility function.
 */
inline std::string datetime_now_string() {
  auto time = std::time(nullptr);
  auto datetime = *std::localtime(&time);
  std::ostringstream output;
  output << std::put_time(&datetime, "%Y-%m-%d_%H%M%S");
  return output.str();
}

}  // namespace upkie::utils
