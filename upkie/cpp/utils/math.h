// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#pragma once

#include <cstdint>

//! Utility functions.
namespace upkie::utils {

//! True if and only if \p divisor divides \p number.
inline bool divides(uint32_t number, uint32_t divisor) {
  if (divisor == 0) {
    return false;
  }
  uint32_t k = number / divisor;
  return (number == k * divisor);
}

}  // namespace upkie::utils
