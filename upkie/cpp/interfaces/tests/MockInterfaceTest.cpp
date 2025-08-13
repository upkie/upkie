// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include <map>

#include "gtest/gtest.h"
#include "upkie/cpp/interfaces/MockInterface.h"
#include "upkie/cpp/interfaces/moteus/protocol.h"

namespace upkie::cpp::interfaces {

TEST(MockInterfaceTest, CycleCallsCallback) {
  const double dt = 1e-3;
  MockInterface interface(dt);

  bool callback_called = false;
  interface.cycle([&callback_called](const moteus::Output& output) {
    callback_called = true;
  });
  ASSERT_TRUE(callback_called);
}

}  // namespace upkie::cpp::interfaces
