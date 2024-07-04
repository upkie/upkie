// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include <map>

#include "gtest/gtest.h"
#include "upkie/cpp/actuation/MockInterface.h"
#include "upkie/cpp/actuation/moteus/protocol.h"
#include "upkie/cpp/actuation/tests/coffee_machine_layout.h"

namespace upkie::actuation {

TEST(MockInterfaceTest, CycleCallsCallback) {
  const auto layout = get_coffee_machine_layout();
  const double dt = 1e-3;
  MockInterface interface(layout, dt);

  bool callback_called = false;
  interface.cycle([&callback_called](const moteus::Output& output) {
    callback_called = true;
  });
  ASSERT_TRUE(callback_called);
}

}  // namespace upkie::actuation
