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

namespace upkie::cpp::actuation {

namespace moteus {

//! Output information forwarded to an interface's callback function.
struct Output {
  //! Size of the replies array.
  size_t query_result_size = 0;
};

}  // namespace moteus

}  // namespace upkie::cpp::actuation
