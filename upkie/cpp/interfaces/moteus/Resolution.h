// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron
/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *     Copyright 2020 Josh Pieper, jjp@pobox.com.
 *     SPDX-License-Identifier: Apache-2.0
 */

#pragma once

namespace upkie::cpp::interfaces {

namespace moteus {

enum class Resolution {
  kInt8,
  kInt16,
  kInt32,
  kFloat,
  kIgnore,
};

}  // namespace moteus

}  // namespace upkie::cpp::interfaces
