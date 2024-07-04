// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/cpp/observers/observe_time.h"

#include "gtest/gtest.h"

namespace upkie::actuation {

TEST(Time, ObserveTime) {
  Dictionary observation;
  observe_time(observation);
  ASSERT_GT(observation.get<double>("time"), 1634311511.42);
}

}  // namespace upkie::actuation
