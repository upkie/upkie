// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace upkie::cpp::interfaces {

namespace moteus {

//! Data resolution types in the moteus protocol.
enum class Resolution {
  kInt8,
  kInt16,
  kInt32,
  kFloat,
  kIgnore,
};

}  // namespace moteus

}  // namespace upkie::cpp::interfaces
