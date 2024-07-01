// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <algorithm>
#include <random>
#include <string>

namespace upkie {

/*! Generate a random string.
 *
 * \param[in] length String length.
 *
 * \return Random string.
 *
 * The generated string contains only alphanumeric characters with no
 * repetition.
 */
inline std::string random_string(unsigned length = 16) {
  std::string alphanum(
      "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz");
  length = (length <= 62) ? length : 62;  // 62 == alphanum.size()
  std::random_device device;
  std::mt19937 generator(device());
  std::shuffle(alphanum.begin(), alphanum.end(), generator);
  return alphanum.substr(0, length);
}

}  // namespace upkie
