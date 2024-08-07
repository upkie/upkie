// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "upkie/cpp/observers/Observer.h"

namespace upkie::cpp::observers {

TEST(Observer, BaseClassObservesNothing) {
  Observer observer;
  Dictionary observation;
  ASSERT_TRUE(observation.is_empty());
  observer.read(observation);
  observer.write(observation);
  ASSERT_TRUE(observation.is_empty());
}

}  // namespace upkie::cpp::observers
