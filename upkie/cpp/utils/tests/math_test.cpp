// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/cpp/utils/math.h"

#include "gtest/gtest.h"

namespace upkie {

TEST(Math, Divides) {
  ASSERT_FALSE(divides(1000000u, 0u));
  ASSERT_FALSE(divides(100000u, 42u));
  ASSERT_TRUE(divides(100u, 20u));
}

}  // namespace upkie
