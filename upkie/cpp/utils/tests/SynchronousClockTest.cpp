// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "gtest/gtest.h"
#include "upkie/cpp/utils/SynchronousClock.h"

namespace upkie::utils {

TEST(SynchronousClock, GetsSomeSleep) {
  SynchronousClock clock(10 /* Hz */);  // 100 ms loop
  ASSERT_DOUBLE_EQ(clock.slack(), 0.);
  for (unsigned i = 0; i < 10; ++i) {
    clock.wait_for_next_tick();
  }
  if (clock.skip_count() < 1) {
    ASSERT_GT(clock.slack(), 1e-4);  // last sleep duration was positive
  } else /* (clock.skip_count() >= 1) */ {
    ASSERT_DOUBLE_EQ(clock.slack(), 0.);
  }
}

TEST(SynchronousClock, NoMarginWhenSkippingCycles) {
  SynchronousClock clock(100 /* Hz */);  // 10 ms loop
  ASSERT_DOUBLE_EQ(clock.slack(), 0.);
  clock.wait_for_next_tick();
  ASSERT_GT(clock.slack(), 0.001);  // 10% of 10 ms loop
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  clock.wait_for_next_tick();
  ASSERT_DOUBLE_EQ(clock.slack(), 0.);
}

}  // namespace upkie::utils
